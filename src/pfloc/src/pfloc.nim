import rclnim
import rclnim/chronossupport
from std/times import nil
import chronos
import std/math
importInterface nav_msgs/msg/odometry
importInterface tf2_msgs/msg/tf_message
importInterface geometry_msgs/msg/[vector3, quaternion, transform, transform_stamped]
importInterface std_msgs/msg/header
importInterface builtin_interfaces/msg/time, Time as TimeMsg
import vmath

proc onMessage[T](sub: Subscription[T], cb: proc(msg: sink T) {.async.}): Future[void] =
  proc loop() {.async.} =
    while true:
      let msg = await sub.recv()
      await cb(msg)
  loop()

type
  Pose = object
    pos: Vec2
    yaw: float32

  PflocNode = ref object
    node: Node
    odomSub: Subscription[Odometry]
    odomFilteredPub: Publisher[Odometry]
    tfPub: Publisher[TFMessage]
    lastOdom: Odometry
    roboPose: Pose
    lastLoopRun: Moment
    timerCallback: TimerCallback
    loopFut: Future[void]

proc new(_: typedesc[PflocNode]): PflocNode =
  result = PflocNode()
  result.node = Node.new("pfloc")
  result.odomSub = result.node.createSubscription(Odometry, "odom", SensorDataQoS)
  result.odomFilteredPub = result.node.createPublisher(Odometry, "odom_filtered", SensorDataQoS)
  result.tfPub = result.node.createPublisher(TFMessage, "/tf", SystemDefaultQoS.withPolicies(depth = 100))

func quatFromAxisAngle*(x, y, z, angle: float): Quaternion =
  let d = sqrt(x * x + y * y + z * z)
  let s = sin(angle * 0.5) / d
  Quaternion(x: x * s, y: y * s, z: z * s, w: cos(angle * 0.5))

proc loop(self: PflocNode) {.async.} =
  let now = Moment.now()
  let dt = now - self.lastLoopRun
  self.lastLoopRun = now
  # echo "loop"

  let dtSec = float(dt.nanoseconds) * 1e-9
  let vel = rotateZ(-self.roboPose.yaw) * vec3(self.lastOdom.twist.twist.linear.x, self.lastOdom.twist.twist.linear.y, self.lastOdom.twist.twist.linear.z)
  self.roboPose.pos += vel.xy * dtSec
  self.roboPose.yaw += self.lastOdom.twist.twist.angular.z * dtSec

  let stamp = times.`+`(times.getTime(), times.initDuration(milliseconds=0))
  var tf = TransformStamped(
    header: Header(frameId: "odom", stamp: TimeMsg(sec: times.toUnix(stamp).int32, nanosec: times.nanosecond(stamp).uint32)),
    childFrameId: "base_footprint",
    transform: Transform(
      translation: Vector3(x: self.roboPose.pos.x, y: self.roboPose.pos.y),
      rotation: quatFromAxisAngle(0, 0, 1, self.roboPose.yaw)
    )
  )
  self.tfPub.publish(TFMessage(transforms: @[tf]))

  var filtered = self.lastOdom
  filtered.pose.pose.position.x = self.roboPose.pos.x
  filtered.pose.pose.position.y = self.roboPose.pos.y
  filtered.pose.pose.orientation = quatFromAxisAngle(0, 0, 1, self.roboPose.yaw)
  self.odomFilteredPub.publish(filtered)

proc run(self: PflocNode) {.async.} =
  var futs = newSeq[Future[void]]()
  futs.add self.odomSub.onMessage(
    proc(msg: sink Odometry) {.async.} =
      # echo msg
      self.lastOdom = msg
  )

  proc loopAux(udata: pointer) {.gcsafe, raises: [].} =
    self.timerCallback = setTimer(Moment.now() + 10.milliseconds, loopAux, nil)
    self.loopFut = loop(self)
  
  loopAux(nil)

  discard await one futs
  for fut in futs:
    await fut.cancelAndWait()
  self.timerCallback.clearTimer()

proc main() {.async.} =
  let node = PflocNode.new()
  await node.run()

when isMainModule:
  rclnim.init()
  waitFor main()
