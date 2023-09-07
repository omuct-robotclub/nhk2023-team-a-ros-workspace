import rclnim
import rclnim/chronossupport
import chronos
import std/[options, strformat, with, strutils, tables]
import ./socketcan
import vmath
import ./protocols
import ./objparamservers

importInterface geometry_msgs/msg/[twist, vector3]
importInterface nav_msgs/msg/odometry, Odometry as OdometryMsg
importInterface std_msgs/msg/[int8 as int8_msg, float64 as float64_msg, bool as bool_msg], bool_msg.Bool as BoolMsg
importInterface std_srvs/srv/trigger

func toDurationSec(f: float): Duration =
  int(f*1e9).nanoseconds

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
    steer0, steer1, steer2, steer3: SteerParameters

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
)

  #   declare("drive.kp", 1.0)
  #   declare("drive.ki", 4.0)
  #   declare("drive.kd", 0.0)
  #   declare("drive.max", 20.0)
  #   declare("drive.min", -20.0)
  #   declare("drive.use_velocity_for_d_term", false)
  #   declare("drive.antiwindup", true)

  #   declare("steer.kp", 2.0)
  #   declare("steer.ki", 0.8)
  #   declare("steer.kd", 0.0)
  #   declare("steer.max", 0.9)
  #   declare("steer.min", -0.9)
  #   declare("steer.use_velocity_for_d_term", true)
  #   declare("steer.antiwindup", false)

  #   declare("steer0.offset", 0.0)
  #   declare("steer1.offset", 0.0)
  #   declare("steer2.offset", 0.0)
  #   declare("steer3.offset", 0.0)

  #   declare("can_id", 200)
  #   declare("can_interface", "can0")
  #   declare("command_lifespan_sec", 1.0)
  #   declare("linear_accel_limit", 10.0)
  #   declare("angular_accel_limit", 10.0)


type CanBridgeNode = ref object
  node: Node
  can: CanSocket
  timer: AsyncFD
  params: ObjParamServer[Parameters]
  lastCmdTime: Moment
  presentLinearVel: Vec2
  presentAngVel: float
  targetLinearVel: Vec2
  targetAngVel: float

  cmdVelSub: Subscription[Twist]
  cmdVelFilteredPub: Publisher[Twist]
  unwindSrv: Service[Trigger]
  donfanCmdSub: Subscription[Int8]
  expanderLengthSub: Subscription[Float64]
  collectorCmdSub: Subscription[BoolMsg]
  armLengthSub: Subscription[Float64]
  armAngleSub: Subscription[Float64]
  largeWheelCmdSub: Subscription[Float64]

  odomPub: Publisher[OdometryMsg]

  roboParamEventQueue: AsyncQueue[ParamEventObj]
  roboGetParamResponseQueue: AsyncQueue[GetParamResponseObj]
  roboSteerUnwindDoneQueue: AsyncQueue[bool]
  roboState: RoboState
  roboStateEvent: AsyncEvent

  canOpenedEvent: AsyncEvent
  canWriteQueue: AsyncQueue[(CANFrame, Future[void])]

using self: CanBridgeNode

func logger(self): Logger =
  self.node.getLogger()

proc newCanBridgeNode(): CanBridgeNode =
  new result
  result.node = newNode("can_bridge")
  result.params = result.node.createObjParamServer(DefaultParameter)
  result.cmdVelSub = result.node.createSubscription(Twist, "cmd_vel", SystemDefaultQoS)
  result.cmdVelFilteredPub = result.node.createPublisher(Twist, "cmd_vel_filtered", SensorDataQoS)
  result.unwindSrv = result.node.createService(Trigger, "unwind", ServiceDefaultQoS)
  result.donfanCmdSub = result.node.createSubscription(Int8, "donfan_cmd", SystemDefaultQoS)
  result.expanderLengthSub = result.node.createSubscription(Float64, "expander_length", SystemDefaultQoS)
  result.collectorCmdSub = result.node.createSubscription(BoolMsg, "collector_cmd", SystemDefaultQoS)
  result.armLengthSub = result.node.createSubscription(Float64, "arm_length", SystemDefaultQoS)
  result.armAngleSub = result.node.createSubscription(Float64, "arm_angle", SystemDefaultQoS)
  result.largeWheelCmdSub = result.node.createSubscription(Float64, "large_wheel_cmd", SystemDefaultQoS)
  result.odomPub = result.node.createPublisher(OdometryMsg, "odom", SensorDataQoS)

  result.roboParamEventQueue = newAsyncQueue[ParamEventObj]()
  result.roboGetParamResponseQueue = newAsyncQueue[GetParamResponseObj]()
  result.roboSteerUnwindDoneQueue = newAsyncQueue[bool]()
  result.roboState = Unknown
  result.roboStateEvent = newAsyncEvent()

  result.canOpenedEvent = newAsyncEvent()
  result.canWriteQueue = newAsyncQueue[(CANFrame, Future[void])]()

proc sendCmd*(self; cmd: RoboCmd): Future[void] =
  let cmdData = RoboCmdData(msg: cmd)
  let frame = CANFrame(
    id: self.params.value.can_id.CANId,
    kind: Data, format: Standard, len: 8, data: cmdData.bytes
  )
  result = newFuture[void]("sendCmd")
  self.canWriteQueue.putNoWait((frame, result))
  # await self.can.write(frame)

proc sendParameter(self; p: SetParamObj) {.async.} =
  let cmd = RoboCmd(
    kind: SetParam,
    setParam: p
  )
  self.roboParamEventQueue.clear()
  while true:
    echo cmd
    await self.sendCmd(cmd)
    let fut = self.roboParamEventQueue.get()
    if await withTimeout(fut, 100.milliseconds):
      let ev = fut.read()
      if ev.id == p.id:
        break
      else:
        self.logger.warn "failed to sync parameter ", p.id

proc syncParameter(self; name: string, value: ParamValue) {.async.} =
  # TODO: refactor this shit
  let pidParamOffsetLut = {
    "kp": 0, "ki": 1, "kd": 2,
    "max": 3, "min": 4, "antiwindup": 5, "use_velocity_for_d_term": 6}.toTable
  let s = name.split(".")
  if s.len < 1: return
  self.logger.info "sending parameter ", name
  if name.startsWith("drive.") or name.startsWith("steer."):
    if s.len != 2: return
    if s[1] notin pidParamOffsetLut: return
    var paramId =
      if s[0] == "drive": DRIVE_KP
      else: STEER_KP
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
  await sleepAsync 10000.milliseconds
  # while true:
  #   for ev in await self.params.eventQueue.waitEvents(key):
  #     if ev.kind == Changed:
  #       echo ev
  #       await self.syncParameter(ev.name, ev.param)

proc cmdSubLoop(self) {.async.} =
  while true:
    let cmd = await self.cmdVelSub.recv()
    self.lastCmdTime = Moment.now()
    self.targetLinearVel.x = cmd.linear.x
    self.targetLinearVel.y = cmd.linear.y
    self.targetAngVel = cmd.angular.z

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

    # let success = await self.roboSteerUnwindDoneQueue.get()
    # echo "waiting queue"
    # echo "got"
    # self.logger.info "unwinding done"

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
    await self.sendCmd(RoboCmd(kind: SetExpanderLength, setExpanderLength: SetExpanderLengthObj(length: int16(cmd.data / 1000.0))))

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
      RoboCmd(kind: SetArmLength, setArmLength: SetArmLengthObj(length: int16(cmd.data / 1000.0))))

proc armAngleSubLoop(self) {.async.} =
  while true:
    let cmd = await self.armAngleSub.recv()
    self.logger.info "arm angle: ", cmd.data
    await self.sendCmd(
      RoboCmd(kind: SetArmAngle, setArmAngle: SetArmAngleObj(angle: int16(cmd.data / 1000.0))))

proc openCan(self) {.async.} =
  while true:
    if self.can == nil or not self.can.isOpened:
      let name = self.params.value.can_interface
      try:
        self.can = createCANSocket(name)
        self.canOpenedEvent.fire()
        self.logger.info fmt"can interface '{name}' opened"
        return
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
      except OSError:
        self.can.close()
        self.canOpenedEvent.clear()

    if frame.id != (self.params.value.can_id + 1).CANId: continue
    if frame.kind != Data or frame.format != Standard or frame.len != 8: continue
    var fb = RoboFeedback()
    copyMem(addr fb, addr frame.data, sizeof(fb))
    echo fb
    case fb.kind
    of PARAM_EVENT: await self.roboParamEventQueue.put(fb.paramEvent)
    of GET_PARAM_RESPONSE: await self.roboGetParamResponseQueue.put(fb.getParamResponse)
    of ODOMETRY:
      var msg = OdometryMsg()
      msg.header.frameId = "odom"
      msg.header.stamp = self.node.clock.now().toMsg()
      msg.childFrameId = "base_footprint"
      msg.twist.twist.linear.x = float(fb.odometry.vx) * 1e-3
      msg.twist.twist.linear.y = float(fb.odometry.vy) * 1e-3
      msg.twist.twist.angular.z = float(fb.odometry.angVel) * 1e-3
      self.odomPub.publish(msg)
    of HEARTBEAT: discard
    of STEER_UNWIND_DONE: await self.roboSteerUnwindDoneQueue.put(true)
    of CURRENT_STATE:
      # self.logger.info fb.currentState.state
      case fb.currentState.state
      of Configuring:
        self.roboState = Configuring
        self.roboStateEvent.fire()
      of Running:
        if self.roboState != Running:
          self.roboState = Running
          self.roboStateEvent.fire()

proc canWriteLoop(self) {.async.} =
  while true:
    let (frame, fut) = await self.canWriteQueue.get()
    while true:
      await self.canOpenedEvent.wait()
      try:
        await self.can.write(frame)
        fut.complete()
        break
      except OSError:
        self.can.close()
        self.canOpenedEvent.clear()

proc updateVelocity(self; dt: Duration) =
  let dtSec = dt.nanoseconds.float * 1e-9

  block:
    let
      tgt = self.targetLinearVel
      cur = self.presentLinearVel
      lim = self.params.value.linear_accel_limit * dtSec
    if tgt.dist(cur) > lim:
      let dir = normalize(tgt - cur)
      self.presentLinearVel += dir * lim
    else:
      self.presentLinearVel = tgt

  block:
    let
      tgt = self.targetAngVel
      cur = self.presentAngVel
      lim = self.params.value.angular_accel_limit * dtSec
    if abs(tgt - cur) > lim:
      let dir = sign(tgt - cur)
      self.presentAngVel += dir * lim
    else:
      self.presentAngVel = tgt
  
  self.cmdVelFilteredPub.publish(Twist(
      linear: Vector3(x: self.presentLinearVel.x, y: self.presentLinearVel.y),
      angular: Vector3(z: self.presentAngVel)))

proc writeVelocity(self) {.async.} =
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
    await self.writeVelocity()
    await sleep

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
      await self.roboStateEvent.wait()
      self.roboStateEvent.clear()
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
    self.largeWheelCmdSubLoop(),
    self.unwindSrvLoop(),
    self.donfanCmdSubLoop(),
    self.expanderCmdSubLoop(),
    self.collectorCmdSubLoop(),
    self.armLengthSubLoop(),
    self.armAngleSubLoop(),
    self.velCmdLoop(),
    self.roboSetupLoop(),
    self.canReadLoop(),
    self.canWriteLoop(),
    self.shutdownChecker(),
  ]

  discard await one tasks

  echo "Shutting down"
  for task in tasks:
    task.cancel()

proc main =
  rclnim.init()
  let node = newCanBridgeNode()
  waitFor node.run()

main()
