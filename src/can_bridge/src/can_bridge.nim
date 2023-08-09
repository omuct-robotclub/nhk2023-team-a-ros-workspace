import rclnim
import rclnim/chronossupport
import chronos
import std/[options, strformat]
import socketcan
import vmath
import ./protocols

importInterface geometry_msgs/msg/[twist, vector3]
importInterface std_msgs/msg/float64
importInterface robot_interface/srv/get_over, GetOver as GetOverSrv
importInterface robot_interface/msg/arm_command

func toDurationSec(f: float): Duration =
  int(f*1e9).nanoseconds

type CanBridgeNode = ref object
  node: Node
  can: CanSocket
  timer: AsyncFD
  params: ParamServer
  lastCmdTime: Moment
  presentLinearVel: Vec2
  presentAngVel: float
  targetLinearVel: Vec2
  targetAngVel: float
  cmdVelSub: Subscription[Twist]
  cmdVelFilteredPub: Publisher[Twist]
  getOverSrv: Service[GetOverSrv]
  donfanCmdSub: Subscription[Float64]
  expanderCmdSub: Subscription[Float64]
  collectorCmdSub: Subscription[Float64]
  armCmdSub: Subscription[ArmCommand]

using self: CanBridgeNode

proc newCanBridgeNode*(): CanBridgeNode =
  new result
  result.can = createCANSocket("vcan0")
  result.node = newNode("can_bridge")
  result.params = result.node.createParamServer()
  result.cmdVelSub = result.node.createSubscription(Twist, "cmd_vel", SystemDefaultQoS)
  result.cmdVelFilteredPub = result.node.createPublisher(Twist, "cmd_vel_filtered", SystemDefaultQoS)
  result.getOverSrv = result.node.createService(GetOverSrv, "get_over", SystemDefaultQoS)
  result.donfanCmdSub = result.node.createSubscription(Float64, "donfan_cmd", SystemDefaultQoS)
  result.expanderCmdSub = result.node.createSubscription(Float64, "expander_cmd", SystemDefaultQoS)
  result.collectorCmdSub = result.node.createSubscription(Float64, "collector_cmd", SystemDefaultQoS)
  result.armCmdSub = result.node.createSubscription(ArmCommand, "arm_cmd", SystemDefaultQoS)
  result.params.declare("pid_x.kp", 0.0)
  result.params.declare("can_id", 144)
  result.params.declare("command_lifespan_sec", 1.0)
  result.params.declare("linear_accel_limit", 3.0)
  result.params.declare("angular_accel_limit", 3.0)
  result.params.endDeclaration()

proc sendCmd*(self; cmd: RoboCmd, doRetry = true, interval: Duration = 10.milliseconds) {.async.} =
  let cmdData = RoboCmdData(msg: cmd)
  let frame = CANFrame(
    id: self.params.get("can_id").intVal.CANId,
    kind: Data, format: Standard, len: 8, data: cmdData.bytes
  )
  if doRetry:
    while not self.can.write(frame):
      self.node.getLogger.warn fmt"failed to send command. retrying after {interval}"
      await sleepAsync interval
  else:
    discard self.can.write(frame)

proc paramEventLoop(self) {.async.} =
  while true:
    for ev in await self.params.waitForUpdate():
      if ev.kind != Changed: continue
      let paramId =
        case ev.name
        of "pid_x.kp": some X_Kp
        of "pid_x.ki": some X_Ki
        of "pid_x.kd": some X_Kd
        of "pid_y.kp": some Y_Kp
        of "pid_y.ki": some Y_Ki
        of "pid_y.kd": some Y_Kd
        of "pid_yaw.kp": some Yaw_Kp
        of "pid_yaw.ki": some Yaw_Ki
        of "pid_yaw.kd": some Yaw_Kd
        else: none(RoboParamId)
      if paramId.isSome:
        let cmd = RoboCmd(
          kind: SetParam,
          setParam: SetParamObj(
            id: paramId.get,
            value: RoboParamValue(kind: Float, floatVal: ev.param.doubleVal)
          )
        )
        await self.sendCmd(cmd)

proc cmdSubLoop(self) {.async.} =
  while true:
    let cmd = await self.cmdVelSub.recv()
    self.lastCmdTime = Moment.now()
    self.targetLinearVel.x = cmd.linear.x
    self.targetLinearVel.y = cmd.linear.y
    self.targetAngVel = cmd.angular.z

proc getOverSrvLoop(self) {.async.} =
  while true:
    let (req, sender) = await self.getOverSrv.recv()
    await self.sendCmd(RoboCmd(
      kind: GetOver,
      getOver: GetOverObj(
        stepKind: req.stepKind.StepKind
    )))
    sender.send(GetOverResponse())

proc donfanCmdSubLoop(self) {.async.} =
  while true:
    let cmd = await self.donfanCmdSub.recv()
    await self.sendCmd(RoboCmd(kind: SetDonfanCmd, setDonfanCmd: SetDonfanCmdObj(cmd: int16(cmd.data * int16.high.float64))))

proc expanderCmdSubLoop(self) {.async.} =
  while true:
    let cmd = await self.expanderCmdSub.recv()
    await self.sendCmd(RoboCmd(kind: SetExpanderCmd, setExpanderCmd: SetExpanderCmdObj(cmd: int16(cmd.data * int16.high.float64))))

proc collectorCmdSubLoop(self) {.async.} =
  while true:
    let cmd = await self.collectorCmdSub.recv()
    await self.sendCmd(RoboCmd(kind: SetCollectorCmd, setCollectorCmd: SetCollectorCmdObj(cmd: int16(cmd.data * int16.high.float64))))

proc armCmdSubLoop(self) {.async.} =
  while true:
    let cmd = await self.armCmdSub.recv()
    await self.sendCmd(
      RoboCmd(kind: SetArmCmd, setArmCmd: SetArmCmdObj(
        expanderCmd: int16(cmd.expanderCommand * int16.high.float64),
        tiltCmd: int16(cmd.tiltCommand * int16.high.float64),
      )))

proc updateVelocity(self; dt: Duration) =
  let dtSec = dt.nanoseconds.float * 1e-9
  
  block:
    let
      tgt = self.targetLinearVel
      cur = self.presentLinearVel
      lim = self.params.get("linear_accel_limit").doubleVal * dtSec
    if tgt.dist(cur) > lim:
      let dir = normalize(tgt - cur)
      self.presentLinearVel += dir * lim
    else:
      self.presentLinearVel = tgt
  
  block:
    let
      tgt = self.targetAngVel
      cur = self.presentAngVel
      lim = self.params.get("angular_accel_limit").doubleVal * dtSec
    if abs(tgt - cur) > lim:
      let dir = sign(tgt - cur)
      self.presentAngVel += dir * lim
    else:
      self.presentAngVel = tgt
  
  self.cmdVelFilteredPub.publish(Twist(
      linear: Vector3(x: self.presentLinearVel.x, y: self.presentLinearVel.y),
      angular: Vector3(z: self.presentAngVel)))

proc writeVelocity(self) =
  var frame = CANFrame(
    id: self.params.get("can_id").intVal.CANId,
    kind: Data,
    format: Standard,
  )
  let cmd = RoboCmdData(
    msg: RoboCmd(
      kind: SetTargetVelocity,
      setTargetVelocity: SetTargetVelocityObj(
        vx: int16 self.presentLinearVel.x * 1000,
        vy: int16 self.presentLinearVel.y * 1000,
        angVel: int16 self.presentAngVel * 1000,
      )
    )
  )
  frame.len = 8
  frame.data = cmd.bytes
  discard self.can.write(frame)

proc loop(self) {.async.} =
  var prevLoopRun = Moment.now() - 10.milliseconds
  while true:
    let sleep = sleepAsync(10.milliseconds)
    let now = Moment.now()
    let dt = now - prevLoopRun
    prevLoopRun = now
    let cmdLifespan = self.params.get("command_lifespan_sec").doubleVal.toDurationSec()
    if now - self.lastCmdTime > cmdLifespan:
      self.targetLinearVel = Vec2.zeroDefault
      self.targetAngVel = 0.0
    self.updateVelocity(dt)
    self.writeVelocity()
    await sleep

proc run(self) {.async.} =
  await allFutures [
    self.paramEventLoop(),
    self.cmdSubLoop(),
    self.getOverSrvLoop(),
    self.donfanCmdSubLoop(),
    self.expanderCmdSubLoop(),
    self.collectorCmdSubLoop(),
    self.armCmdSubLoop(),
    self.loop(),
  ]
  echo "Shutting down"

proc main =
  rclnim.init()
  let node = newCanBridgeNode()
  waitFor node.run()

main()
