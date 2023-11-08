import rclnim
import rclnim/chronossupport
import chronos
import std/[options, strformat, strutils, tables, oserrors]
import ./socketcan
import vmath
import ./protocols
import tf2/[tfbuffers, msgconverters]
import geonimetry

importInterface geometry_msgs/msg/[twist, twist_stamped, vector3, quaternion]
importInterface nav_msgs/msg/odometry, Odometry as OdometryMsg
importInterface std_msgs/msg/[int8 as int8_msg, float64 as float64_msg, bool as bool_msg], bool_msg.Bool as BoolMsg
importInterface std_srvs/srv/trigger
importInterface robot_interface/msg/steer_unit_states

func toDurationSec(f: float): Duration =
  int(f*1e9).nanoseconds

proc withTimeoutCancel[T](fut: Future[T], timeout: Duration): Future[bool] {.async.} =
  if await withTimeout(fut, timeout):
    return true
  else:
    await fut.cancelAndWait()
    return fut.completed()

type RoboState = enum
  Unknown
  Configuring
  Running

type
  PidParameters = object
    kp, ki, kd, max, min: float64
    use_velocity_for_d_term, antiwindup: bool
  
  SteerParameters = object
    offset: float64

  Parameters = object
    can_id: int64
    can_interface: string
    command_lifespan_sec: float64
    linear_accel_limit: float64
    angular_accel_limit: float64

    drive: PidParameters
    steer: PidParameters
    arm_angle: PidParameters
    arm_length: PidParameters
    expander: PidParameters
    steer0, steer1, steer2, steer3: SteerParameters

    lock_arm_angle: bool

const DefaultParameter = Parameters(
  can_id: 200,
  can_interface: "can0",
  command_lifespan_sec: 1.0,
  linear_accel_limit: 10.0,
  angular_accel_limit: 10.0,
  drive: PidParameters(
    kp: 1.0, ki: 4.0, kd: 0.0, max: 20.0, min: -20.0, use_velocity_for_d_term: false, antiwindup: true
  ),
  steer: PidParameters(
    kp: 2.0, ki: 0.8, kd: 0.0, max: 0.9, min: -0.9, use_velocity_for_d_term: true, antiwindup: false
  ),
  lock_arm_angle: false
)


type CanBridgeNode = ref object
  node: Node
  buf: TfBuffer
  can: CanSocket
  timer: AsyncFD
  params: ObjParamServer[Parameters]
  lastCmdTime: Moment
  presentLinearVel: Vec2
  presentAngVel: float
  targetLinearVel: Vec2
  targetAngVel: float

  prevOdomLinearVel: Vector2d
  prevOdomAngVel: float

  prevPose: PoseObj
  position: Vector2d
  rotation: float

  cmdVelSub: Subscription[Twist]
  cmdVelStampedSub: Subscription[TwistStamped]
  cmdVelFilteredPub: Publisher[Twist]
  unwindSrv: Service[Trigger]
  donfanCmdSub: Subscription[Int8]
  expanderLengthSub: Subscription[Float64]
  collectorCmdSub: Subscription[BoolMsg]
  armLengthSub: Subscription[Float64]
  armAngleSub: Subscription[Float64]
  largeWheelCmdSub: Subscription[Float64]

  odomPub: Publisher[OdometryMsg]
  steerUnitStatePub: Publisher[SteerUnitStates]

  roboParamEventQueue: AsyncQueue[ParamEventObj]
  roboGetParamResponseQueue: AsyncQueue[GetParamResponseObj]
  roboSteerUnwindDoneQueue: AsyncQueue[bool]
  roboState: RoboState
  roboStateEvent: AsyncEvent
  steerStates: SteerUnitStates

  canOpenedEvent: AsyncEvent
  canWriteQueue: AsyncQueue[(CANFrame, Future[void])]

  velocityUpdatedEvent: AsyncEvent

  noBufferSpaceHasWarned: bool

using self: CanBridgeNode

func logger(self): Logger =
  self.node.getLogger()

proc newCanBridgeNode(): CanBridgeNode =
  new result
  result.node = newNode("can_bridge")
  result.buf = newTfBuffer(result.node)
  result.params = result.node.createObjParamServer(DefaultParameter)
  result.cmdVelSub = result.node.createSubscription(Twist, "cmd_vel", SystemDefaultQoS)
  result.cmdVelStampedSub = result.node.createSubscription(TwistStamped, "cmd_vel_stamped", SystemDefaultQoS)
  result.cmdVelFilteredPub = result.node.createPublisher(Twist, "cmd_vel_filtered", SensorDataQoS)
  result.unwindSrv = result.node.createService(Trigger, "unwind", ServiceDefaultQoS)
  result.donfanCmdSub = result.node.createSubscription(Int8, "donfan_cmd", SystemDefaultQoS)
  result.expanderLengthSub = result.node.createSubscription(Float64, "expander_length", SystemDefaultQoS)
  result.collectorCmdSub = result.node.createSubscription(BoolMsg, "collector_cmd", SystemDefaultQoS)
  result.armLengthSub = result.node.createSubscription(Float64, "arm_length", SystemDefaultQoS)
  result.armAngleSub = result.node.createSubscription(Float64, "arm_angle", SystemDefaultQoS)
  result.largeWheelCmdSub = result.node.createSubscription(Float64, "large_wheel_cmd", SystemDefaultQoS)
  result.odomPub = result.node.createPublisher(OdometryMsg, "odom", SensorDataQoS)
  result.steerUnitStatePub = result.node.createPublisher(SteerUnitStates, "steer_states", SensorDataQoS)

  result.roboParamEventQueue = newAsyncQueue[ParamEventObj](1000)
  result.roboGetParamResponseQueue = newAsyncQueue[GetParamResponseObj](1000)
  result.roboSteerUnwindDoneQueue = newAsyncQueue[bool](1000)
  result.roboState = Unknown
  result.roboStateEvent = newAsyncEvent()
  result.steerStates = SteerUnitStates(
    angles: @[0, 0, 0, 0],
    currents: @[0, 0, 0, 0],
    velocities: @[0, 0, 0, 0]
  )

  result.canOpenedEvent = newAsyncEvent()
  result.canWriteQueue = newAsyncQueue[(CANFrame, Future[void])](1000)

  result.velocityUpdatedEvent = newAsyncEvent()

proc sendCmd*(self; cmd: RoboCmd) {.async.} =
  let cmdData = RoboCmdData(msg: cmd)
  let frame = CANFrame(
    id: self.params.value.can_id.CANId,
    kind: Data, format: Standard, len: 8, data: cmdData.bytes
  )
  let fut = newFuture[void]("sendCmd")
  await self.canWriteQueue.put((frame, fut))
  await fut
  # await self.can.write(frame)

proc sendParameter(self; p: SetParamObj) {.async.} =
  self.logger.info "sending parameter ", p
  let cmd = RoboCmd(
    kind: SetParam,
    setParam: p
  )
  while true:
    self.roboParamEventQueue.clear()
    self.logger.info "sending set parameter request"
    await self.sendCmd(cmd)
    self.logger.info "set parameter request sent"
    let fut = self.roboParamEventQueue.get()
    self.logger.info "waiting for response"
    if await withTimeoutCancel(fut, 100.milliseconds):
      let ev = fut.read()
      if ev.id == p.id:
        self.logger.info "set parameter success"
        break
      else:
        self.logger.warn "failed to sync parameter ", p.id
        await sleepAsync 100.milliseconds
    else:
      self.logger.info "response timed out"

proc syncParameter(self; name: string, value: ParamValue) {.async.} =
  # TODO: refactor this shit
  let pidParamOffsetLut = {
    "kp": 0, "ki": 1, "kd": 2,
    "max": 3, "min": 4, "antiwindup": 5, "use_velocity_for_d_term": 6}.toTable
  let s = name.split(".")
  if s.len < 1: return
  # self.logger.info "sending parameter ", name
  if name.startsWith("drive.") or
      name.startsWith("steer.") or
      name.startsWith("arm_angle.") or
      name.startsWith("arm_length.") or
      name.startsWith("expander."):
    if s.len != 2: return
    if s[1] notin pidParamOffsetLut: return
    var paramId =
      case s[0]
      of "drive": DRIVE_KP
      of "steer": STEER_KP
      of "arm_angle": ARM_ANGLE_KP
      of "arm_length": ARM_LENGTH_KP
      of "expander": EXPANDER_KP
      else: raiseAssert "unreachable"
    inc paramId, pidParamOffsetLut[s[1]]
    var paramValue: RoboParamValue
    if s[1] == "antiwindup" or s[1] == "use_velocity_for_d_term":
      if value.kind != Bool: return
      paramValue = RoboParamValue(kind: Int, intVal: ord(value.boolVal).int32)
    else:
      paramValue = RoboParamValue(kind: Float, floatVal: value.doubleVal)
    await self.sendParameter(SetParamObj(
      id: paramId,
      value: paramValue))
    await self.sendCmd(RoboCmd(kind: RESET_PID))
  elif s[0] in ["steer0", "steer1", "steer2", "steer3"]:
    if s.len != 2: return
    let paramId = case s[0]
      of "steer0": STEER0_OFFSET
      of "steer1": STEER1_OFFSET
      of "steer2": STEER2_OFFSET
      of "steer3": STEER3_OFFSET
      else: return
    if s[1] != "offset": return
    if value.kind != Double: return
    await self.sendParameter(SetParamObj(
      id: paramId,
      value: RoboParamValue(kind: Float, floatVal: value.doubleVal)))

proc paramEventLoop(self) {.async.} =
  let key = self.params.eventQueue.register()
  while true:
    for ev in await self.params.eventQueue.waitEvents(key):
      if ev.kind == Changed:
        echo ev
        await self.syncParameter(ev.name, ev.param)

proc cmdSubLoop(self) {.async.} =
  while true:
    let cmd = await self.cmdVelSub.recv()
    self.lastCmdTime = Moment.now()
    self.targetLinearVel.x = cmd.linear.x
    self.targetLinearVel.y = cmd.linear.y
    self.targetAngVel = cmd.angular.z

proc cmdStampedSubLoop(self) {.async.} =
  while true:
    let cmd = await self.cmdVelStampedSub.recv()
    let tr = self.buf.lookupTransform(FrameId"base_footprint", cmd.header.frameId.FrameId, TimePointZero)
    if tr.isErr: continue
    let tf = tr.tryValue().transform.to(Transform3d)
    let linear = tf.basis * cmd.twist.linear.to(Vector3d)
    let angular = cmd.twist.angular.to(Vector3d)
    self.lastCmdTime = Moment.now()
    self.targetLinearVel.x = linear.x
    self.targetLinearVel.y = linear.y
    self.targetAngVel = angular.z

proc largeWheelCmdSubLoop(self) {.async.} =
  while true:
    let cmd = await self.largeWheelCmdSub.recv()
    await self.sendCmd(RoboCmd(kind: SetLargeWheelCmd, setLargeWheelCmd: SetLargeWheelCmdObj(cmd: int16(cmd.data * int16.high.float64))))

proc unwindSrvLoop(self) {.async.} =
  while true:
    let (_, sender) = await self.unwindSrv.recv()
    self.logger.info "unwinding"
    await self.sendCmd(RoboCmd(kind: UNWIND_STEER))
    sender.send(TriggerResponse(success: true))

proc donfanCmdSubLoop(self) {.async.} =
  while true:
    let cmd = await self.donfanCmdSub.recv()
    if cmd.data notin [-1, 0, 1]:
      self.logger.warn "donfan: invalid command"
      continue
    self.logger.info "donfan: ", cmd.data
    await self.sendCmd(RoboCmd(kind: SetDonfanCmd, setDonfanCmd: SetDonfanCmdObj(dir: cmd.data)))

proc expanderCmdSubLoop(self) {.async.} =
  while true:
    let cmd = await self.expanderLengthSub.recv()
    self.logger.info "expand: ", cmd.data
    await self.sendCmd(RoboCmd(kind: SetExpanderLength, setExpanderLength: SetExpanderLengthObj(length: int16(cmd.data * 1000.0))))

proc collectorCmdSubLoop(self) {.async.} =
  while true:
    let cmd = await self.collectorCmdSub.recv()
    self.logger.info "collector: ", cmd.data
    await self.sendCmd(RoboCmd(kind: SetCollectorCmd, setCollectorCmd: SetCollectorCmdObj(enable: cmd.data)))

proc armLengthSubLoop(self) {.async.} =
  while true:
    let cmd = await self.armLengthSub.recv()
    self.logger.info "arm length: ", cmd.data
    await self.sendCmd(
      RoboCmd(kind: SetArmLength, setArmLength: SetArmLengthObj(length: int16(cmd.data * 1000.0))))

proc armAngleSubLoop(self) {.async.} =
  while true:
    let cmd = await self.armAngleSub.recv()
    if self.params.value.lock_arm_angle:
      self.logger.warn "An arm angle command received but it's locked."
    else:
      self.logger.info "arm angle: ", cmd.data
      await self.sendCmd(
        RoboCmd(kind: SetArmAngle, setArmAngle: SetArmAngleObj(angle: int16(cmd.data * 1000.0))))

proc openCan(self) {.async.} =
  while true:
    if self.can == nil or not self.can.isOpened:
      let name = self.params.value.can_interface
      try:
        self.can = createCANSocket(name)
        self.canOpenedEvent.fire()
        self.logger.info fmt"can interface '{name}' opened"
      except CatchableError:
        self.logger.warn fmt"failed to open '{name}'"
        if self.node.context.isShuttingDown:
          raise newException(ShutdownError, "shutting down")
    await sleepAsync 1.seconds

proc canReadLoop(self) {.async.} =
  while true:
    var frame: CANFrame
    while true:
      await self.canOpenedEvent.wait()
      try:
        frame = await self.can.read()
        break
      except OSError as e:
        self.logger.warn "Read error: ", e.msg
        self.can.close()
        self.canOpenedEvent.clear()

    if frame.id != (self.params.value.can_id + 1).CANId: continue
    if frame.kind != Data or frame.format != Standard or frame.len < sizeof(RoboFeedback): continue
    var fb = RoboFeedback()
    copyMem(addr fb, addr frame.data, sizeof(fb))
    echo fb
    case fb.kind
    of PARAM_EVENT: await self.roboParamEventQueue.put(fb.paramEvent)
    of GET_PARAM_RESPONSE: await self.roboGetParamResponseQueue.put(fb.getParamResponse)
    of VELOCITY:
      self.prevOdomLinearVel.x = float(fb.velocity.vx) * 1e-3
      self.prevOdomLinearVel.y = float(fb.velocity.vy) * 1e-3
      self.prevOdomAngVel = float(fb.velocity.angVel) * 1e-3
    of POSE:
      # self.logger.info fb.pose, " ", self.prevPose
      var msg = OdometryMsg()
      msg.header.frameId = "odom"
      msg.header.stamp = self.node.clock.now().toMsg()
      msg.childFrameId = "base_footprint"
      self.position.x += float(cast[int16](fb.pose.x - self.prevPose.x)) * 1e-3
      self.position.y += float(cast[int16](fb.pose.y - self.prevPose.y)) * 1e-3
      self.rotation = float(fb.pose.yaw) * 1e-3
      msg.pose.pose.position.x = self.position.x
      msg.pose.pose.position.y = self.position.y
      msg.pose.pose.orientation = quatd(vector3d(0, 0, 1), self.rotation).to(Quaternion)
      msg.twist.twist.linear.x = self.prevOdomLinearVel.x
      msg.twist.twist.linear.y = self.prevOdomLinearVel.y
      msg.twist.twist.angular.z = self.prevOdomAngVel
      self.odomPub.publish(msg)
      self.prevPose = fb.pose
    of HEARTBEAT: discard
    of STEER_UNWIND_DONE: await self.roboSteerUnwindDoneQueue.put(true)
    of CURRENT_STATE:
      case fb.currentState.state
      of Configuring:
        self.roboState = Configuring
        self.roboStateEvent.fire()
      of Running:
        if self.roboState != Running:
          self.roboState = Running
          self.roboStateEvent.fire()
    of STEER_UNIT_STATE:
      doAssert 0 <= fb.steerUnitState.index and fb.steerUnitState.index < 4
      let idx = fb.steerUnitState.index
      self.steerStates.velocities[idx] = fb.steerUnitState.velocity.float64 * 1e-2
      self.steerStates.currents[idx] = fb.steerUnitState.current.float64 * 1e-3
      self.steerStates.angles[idx] = fb.steerUnitState.angle.float64 * 1e-3
    of DETECTED_WALL:
      discard

proc canWriteLoop(self) {.async.} =
  while true:
    let (frame, fut) = await self.canWriteQueue.get()
    while true:
      await self.canOpenedEvent.wait()
      let err = self.can.write(frame)
      if err.isErr:
        let e = err.tryError()
        if e == ENOBUFS:
          if not self.noBufferSpaceHasWarned:
            self.logger.warn "Write error: No buffer space available"
            self.noBufferSpaceHasWarned = true
          await sleepAsync 1000.milliseconds
        else:
          self.logger.warn "Write error: ", e
          self.can.close()
          self.canOpenedEvent.clear()
      else:
        fut.complete()
        self.noBufferSpaceHasWarned = false
        break

proc updateVelocity(self; dt: Duration) =
  let dtSec = dt.nanoseconds.float * 1e-9

  self.presentLinearVel = self.targetLinearVel
  self.presentAngVel = self.targetAngVel
  # block:
  #   let
  #     tgt = self.targetLinearVel
  #     cur = self.presentLinearVel
  #     lim = self.params.value.linear_accel_limit * dtSec
  #   if tgt.dist(cur) > lim:
  #     let dir = normalize(tgt - cur)
  #     self.presentLinearVel += dir * lim
  #   else:
  #     self.presentLinearVel = tgt

  # block:
  #   let
  #     tgt = self.targetAngVel
  #     cur = self.presentAngVel
  #     lim = self.params.value.angular_accel_limit * dtSec
  #   if abs(tgt - cur) > lim:
  #     let dir = sign(tgt - cur)
  #     self.presentAngVel += dir * lim
  #   else:
  #     self.presentAngVel = tgt
  
  self.cmdVelFilteredPub.publish(Twist(
      linear: Vector3(x: self.presentLinearVel.x, y: self.presentLinearVel.y),
      angular: Vector3(z: self.presentAngVel)))

proc writeVelocityLoop(self) {.async.} =
  while true:
    await self.velocityUpdatedEvent.wait()
    self.velocityUpdatedEvent.clear()
    if self.can != nil and self.can.isOpened:
      let cmd = RoboCmd(
        kind: SetTargetVelocity,
        setTargetVelocity: SetTargetVelocityObj(
          vx: int16 self.presentLinearVel.x * 1000,
          vy: int16 self.presentLinearVel.y * 1000,
          angVel: int16 self.presentAngVel * 1000,
        )
      )
      await self.sendCmd(cmd)

proc velCmdLoop(self) {.async.} =
  var prevLoopRun = Moment.now() - 10.milliseconds
  while true:
    let sleep = sleepAsync(10.milliseconds)
    let now = Moment.now()
    let dt = now - prevLoopRun
    prevLoopRun = now
    let cmdLifespan = self.params.value.command_lifespan_sec.toDurationSec()
    if now - self.lastCmdTime > cmdLifespan:
      self.targetLinearVel = Vec2.zeroDefault
      self.targetAngVel = 0.0
    self.updateVelocity(dt)
    self.velocityUpdatedEvent.fire()
    await sleep

proc steerStatePublishLoop(self) {.async.} =
  while true:
    self.steerUnitStatePub.publish(self.steerStates)
    await sleepAsync 33.milliseconds

proc roboSetupLoop(self) {.async.} =
  while true:
    await self.roboStateEvent.wait()
    self.roboStateEvent.clear()
    case self.roboState
    of Configuring:
      self.logger.info "setting up"
      for toSync in self.params.server.names:
        await self.syncParameter(toSync, self.params.server.get(toSync))
      await self.sendCmd(RoboCmd(kind: ACTIVATE))
      self.logger.info "activating..."
      await self.roboStateEvent.wait()
      self.roboStateEvent.clear()
      self.logger.info "activated!"
    of Running:
      discard
    of Unknown:
      discard

proc shutdownChecker(self) {.async.} =
  while true:
    if not self.node.context.isValid:
      raise newException(ShutdownError, "shutdown")
    else:
      await sleepAsync 100.milliseconds

proc run(self) {.async.} =
  let tasks = [
    self.openCan(),
    self.paramEventLoop(),
    self.cmdSubLoop(),
    self.cmdStampedSubLoop(),
    self.largeWheelCmdSubLoop(),
    self.unwindSrvLoop(),
    self.donfanCmdSubLoop(),
    self.expanderCmdSubLoop(),
    self.collectorCmdSubLoop(),
    self.armLengthSubLoop(),
    self.armAngleSubLoop(),
    self.velCmdLoop(),
    self.writeVelocityLoop(),
    self.steerStatePublishLoop(),
    self.roboSetupLoop(),
    self.canReadLoop(),
    self.canWriteLoop(),
    self.shutdownChecker(),
  ]

  discard await one tasks

  echo "Shutting down"
  for task in tasks:
    await task.cancelAndWait()

proc main =
  rclnim.init()
  let node = newCanBridgeNode()
  waitFor node.run()

main()
