import rclnim
import rclnim/chronossupport
from std/times import nil
import chronos
import std/[math, with, strutils]
importInterface nav_msgs/msg/[odometry, path], path.Path as PathMsg
importInterface geometry_msgs/msg/[vector3, quaternion, transform, twist]
importInterface std_msgs/msg/header
importInterface builtin_interfaces/msg/time, Time as TimeMsg
import tf2/[tfbuffers, tfbuffercore]
import ./utils
import geonimetry



proc onMessage[T](sub: Subscription[T], cb: proc(msg: sink T) {.async.}): Future[void] =
  proc loop() {.async.} =
    while true:
      let msg = await sub.recv()
      await cb(msg)
  loop()

type
  Decision = enum
    Right
    Left
    Special

  DecisionZone = object
    bbox: AABB2d
    branch: array[Decision, seq[Pose]]

  Pose = object
    pos: Vector2d
    yaw: float32

  PathGeneratorNode = ref object
    node: Node
    params: ParamServer
    buf: TFBuffer
    odomSub: Subscription[Odometry]
    pathSub: Subscription[PathMsg]
    cmdPub: Publisher[Twist]
    lastOdom: Odometry
    roboPose: Pose
    lastLoopRun: Moment
    timerCallback: TimerCallback
    loopFut: Future[void]
    path: seq[Pose]

using self: PathGeneratorNode

proc new(_: typedesc[PathGeneratorNode]): PathGeneratorNode =
  result = PathGeneratorNode()
  result.node = Node.new("path_generator")
  result.params = result.node.createParamServer()
  result.buf = TFBuffer.new(result.node)
  result.odomSub = result.node.createSubscription(Odometry, "odom_filtered", SensorDataQoS)
  result.pathSub = result.node.createSubscription(PathMsg, "path", SystemDefaultQoS.withPolicies(reliability=Reliable, durability=Volatile))
  result.cmdPub = result.node.createPublisher(Twist, "path_following_cmd_vel", SystemDefaultQoS)
  
  with result.params:
    endDeclaration()

proc getYaw[T](q: Quat[T]): float =
  result = arctan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z)

proc loop(self) {.async.} =
  let now = Moment.now()
  let dt = now - self.lastLoopRun
  self.lastLoopRun = now

  let odomTfRes = self.buf.lookupTransform(FrameId"map", FrameId"odom", TimePointZero)
  if odomTfRes.isErr:
    self.node.getLogger.warn odomTfRes.tryError()
    return
  let odomTf = odomTfRes.tryGet().transform.to(Transform3d)
  let pos = odomTf.xform self.lastOdom.pose.pose.position.to(Vector3d)
  let yaw = odomTf.xform(self.lastOdom.pose.pose.orientation.to(Quatd)).getYaw()
  echo pos, " ", yaw

proc transformPath(self; path: PathMsg): seq[Pose] =
  doAssert path.header.frameId == "map"
  for p in path.poses:
    var pose = Pose()
    doAssert p.header.frameId == "map"
    pose.pos.x = p.pose.position.x
    pose.pos.y = p.pose.position.y
    discard p.pose.orientation.to(Quat[float])
    pose.yaw = p.pose.orientation.to(Quatd).getYaw()
    result.add pose

proc paramEventLoop(self) {.async.} =
  while true:
    for ev in await self.params.waitForUpdate():
      if ev.kind != Changed: continue
      # let controller =
      #   if ev.name.startsWith("pid_x."): addr self.pidX
      #   elif ev.name.startsWith("pid_y."): addr self.pidY
      #   elif ev.name.startsWith("pid_yaw."): addr self.pidYaw
      #   else: nil
      # if controller != nil:
      #   let base = ev.name.split(".")[0]
      #   controller[].gain = self.params.getPidGain(base & ".")
      #   controller[].reset()

proc run(self: PathGeneratorNode) {.async.} =
  var futs = @[
    self.paramEventLoop(),
    self.odomSub.onMessage(
      proc(msg: sink Odometry) {.async.} =
        self.lastOdom = msg
    ),
    self.pathSub.onMessage(
      proc(msg: sink PathMsg) {.async.} =
        self.path = self.transformPath(msg)
    ),
  ]

  proc loopAux(udata: pointer) {.gcsafe, raises: [].} =
    self.timerCallback = setTimer(Moment.now() + 10.milliseconds, loopAux, nil)
    self.loopFut = loop(self)

  loopAux(nil)

  discard await one futs
  for fut in futs:
    await fut.cancelAndWait()
  self.timerCallback.clearTimer()

proc main() {.async.} =
  let node = PathGeneratorNode.new()
  await node.run()

when isMainModule:
  rclnim.init()
  waitFor main()
