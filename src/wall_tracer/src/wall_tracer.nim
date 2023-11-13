import rclnim
import rclnim/chronossupport
import tf2
import tf2/msgconverters
import chronos
import geonimetry
import ./[linefitting, pid_controller]
import std/[options, sequtils, math, times, algorithm, deques]
import results

importInterface std_msgs/msg/color_rgba
importInterface sensor_msgs/msg/laser_scan
importInterface visualization_msgs/msg/[marker, marker_array]
importInterface nav_msgs/msg/odometry
importInterface geometry_msgs/msg/[point, twist]
importInterface builtin_interfaces/msg/time as time_msg, Time as TimeMsg

type
  Parameters = object
    enable_turn: bool
    robot_size_x: float
    robot_size_y: float
    lookahead_distance_incourse: float
    lookahead_distance_outcourse: float
    in_course_distance: float
    out_course_distance: float
    line_fitting: SeqRansacConfig
    odom_lifespan: float
    angular_pid_gain: PidGain

  WallTracer = ref object
    node: Node
    params: ObjParamServer[Parameters]
    buf: TfBuffer
    markerPub: Publisher[MarkerArray]
    scanSub: Subscription[LaserScan]
    cmdSub: Subscription[Twist]
    cmdPub: Publisher[Twist]
    odomSub: Subscription[Odometry]

    latestOdom: Odometry
    lastScanTime: RosTime
    detectedLines: seq[Line]
    odomBuffer: Deque[Odom]

    targetLine: Option[Line]
    carrotPoint: Option[Vector2f]
    targetCourse: Course
    turnEnabled: bool

    angleController: PidController

  Course = enum
    InCourse
    OutCourse
  
  Odom = object
    stamp: RosTime
    dx, dy, da: float

using self: WallTracer

proc newWallTracer(): WallTracer =
  result = new WallTracer
  result.node = newNode("wall_tracer")
  result.buf = TfBuffer.new(result.node)
  result.params = result.node.createObjParamServer(Parameters(
    enable_turn: false,
    robot_size_x: 0.7,
    robot_size_y: 0.7,
    lookahead_distance_incourse: 0.5,
    lookahead_distance_outcourse: 0.5,
    in_course_distance: 2.25,
    out_course_distance: 0.75,
    odom_lifespan: 1.0,
    line_fitting: SeqRansacConfig(
      inlierDistanceThreshold: 0.05,
      minLineLength: 1.0,
      stopTotalLineLength: 1.0,
      iterationCount: 1000,
      minDistanceBetweenSamples: 1.0,
    ),
    angular_pid_gain: PidGain(
      kp: 5.0,
      ki: 0.0,
      kd: 0.0,
      max: 5.0,
      min: -5.0,
      useVelocityForDTerm: true,
      antiWindup: true,
    ),
  ))
  result.scanSub = result.node.createSubscription(LaserScan, "scan", SensorDataQoS)
  result.cmdSub = result.node.createSubscription(Twist, "manual_cmd_vel", SystemDefaultQoS)
  result.cmdPub = result.node.createPublisher(Twist, "cmd_vel", SystemDefaultQoS)
  result.markerPub = result.node.createPublisher(MarkerArray, "wall_tracer_markers", SystemDefaultQoS)
  result.odomSub = result.node.createSubscription(Odometry, "odom", SensorDataQoS)
  result.targetCourse = InCourse
  result.angleController = initPidController(result.params.value.angular_pid_gain)

proc to2Point(line: Line): array[2, Point] =
  let c = vector2d(line.point.x, line.point.y)
  let s = c - line.direction * 100.0
  let e = c + line.direction * 100.0
  [Point(x: s.x, y: s.y), Point(x: e.x, y: e.y)]

proc transformScanToPoints(self; scan: LaserScan): Result[seq[Vector2f], TransformError] =
  let frame = FrameId scan.header.frameId
  let stamp = scan.header.stamp
  let tfRes = self.buf.lookupTransform(FrameId"base_link", frame, initTime(stamp.sec, stamp.nanosec))
  if tfRes.isErr:
    return err(tfRes.tryError())
  let tf = tfRes.tryGet().transform.to(Transform3d)
  
  let minX = -self.params.value.robot_size_x / 2
  let maxX = self.params.value.robot_size_x / 2
  let minY = -self.params.value.robot_size_y / 2
  let maxY = self.params.value.robot_size_y / 2

  var res = newSeq[Vector2f]()
  for i, r in scan.ranges:
    if r.isNaN: continue
    let angle = scan.angleMin + i.float * scan.angleIncrement
    let point = tf.xform rotationZ(angle) * vector3f(r, 0, 0)
    if (point.x < minX or maxX < point.x) or (point.y < minY or maxY < point.y):
      res.add vector2f(point.x, point.y)
  ok(res)

proc publishMarkers(self; filtered, ignored, moved: seq[Line], carrotPoint: Option[Vector2f], stamp: TimeMsg) =
  var delAllMarker = Marker()
  var markers = MarkerArray()
  delAllMarker.id = 0
  delAllMarker.`type` = Marker.DELETEALL
  delAllMarker.header.frameId = "base_link"
  delAllMarker.scale.x = 1.0
  delAllMarker.scale.y = 1.0
  delAllMarker.scale.z = 1.0
  markers.markers.add delAllMarker

  const filteredColor = ColorRGBA(r: 1, g: 0, b: 0, a: 1)
  const ignoredColor = ColorRGBA(r: 1, g: 1, b: 0, a: 1)
  const movedColor = ColorRGBA(r: 0, g: 1, b: 1, a: 1)
  const carrotPointColor = ColorRGBA(r: 1, g: 0, b: 1, a: 1)

  let linesWithColor = filtered.mapIt((it, filteredColor)) & ignored.mapIt((it, ignoredColor)) & moved.mapIt((it, movedColor))

  for i, (line, color) in linesWithColor:
    var m = Marker()
    m.header.frameId = "base_link"
    m.header.stamp = stamp
    m.id = int32 markers.markers.len
    m.`type` = Marker.LINE_LIST
    m.action = Marker.ADD
    m.scale.x = 0.05
    m.scale.y = 0.05
    m.scale.z = 0.05
    m.points = @(line.to2Point())
    m.color = color
    m.lifetime.nanosec = 200_000_000
    markers.markers.add m

  if carrotPoint.isSome:
    let p = carrotPoint.get()
    var m = Marker()
    m.header.frameId = "base_link"
    m.header.stamp = stamp
    m.id = int32 markers.markers.len
    m.`type` = Marker.POINTS
    m.action = Marker.ADD
    m.scale.x = 0.2
    m.scale.y = 0.2
    m.scale.z = 0.2
    m.color = carrotPointColor
    m.lifetime.nanosec = 200_000_000
    m.points.add Point(x: p.x, y: p.y, z: 0.0)
    markers.markers.add m

  self.markerPub.publish(markers)

proc filterLine(self; line: Line): bool =
  const vertical = Line.fromPointAndDir(vector2f(0, 0), vector2f(1, 0))
  const horizontal = Line.fromPointAndDir(vector2f(0, 0), vector2f(0, 1))
  var angle = arctan2(line.direction.y, line.direction.x).float
  const quarterPi = PI / 4
  const halfPi = PI / 2
  if angle > halfPi:
    angle -= PI
  elif angle < -halfPi:
    angle += PI
  if angle in -quarterPi .. quarterPi:
    # vertical line
    let p = horizontal.intersection(line)
    p.get().y < 0
  else:
    # horizontal line
    if self.turnEnabled:
      let p = vertical.intersection(line)
      p.get().x > 0
    else:
      false

proc moveParallel(line: Line, delta: float32): Line =
  let
    d = rotationZ(degToRad(90.0'f)) * vector3f(line.direction.x, line.direction.y, 0) * delta
    p1 = line.point + vector2f(d.x, d.y)
    p2 = line.point - vector2f(d.x, d.y)
  if p1.length2 < p2.length2:
    Line.fromPointAndDir(p1, line.direction)
  else:
    Line.fromPointAndDir(p2, line.direction)

proc getCarrotPoint(self; lines: openArray[Line]): Option[(Vector2f, Line)] =
  var candidates = newSeq[(Vector2f, Line)]()
  var lookaheadDist =
    case self.targetCourse
    of InCourse: self.params.value.lookahead_distance_incourse
    of OutCourse: self.params.value.lookahead_distance_outcourse
  for i in 0..<100:
    lookaheadDist += 0.1
    for l in lines:
      let res = l.intersectionToCircle(vector2f(0, 0), lookaheadDist)
      if res.isNone: continue
      let points = res.get().filterIt(it.x > 0)
      if points.len == 0: continue
      for p in points:
        candidates.add (p, l)

    if candidates.len > 0:
      return some candidates.sortedByIt(arctan2(it[0].y, it[0].x))[^1]
  none (Vector2f, Line)

proc scanSubLoop(self) {.async.} =
  while true:
    let msg = await self.scanSub.recv()
    let pointsRes = self.transformScanToPoints(msg)
    if pointsRes.isErr:
      self.node.getLogger.warn "failed to transform laser scan"
      continue
    let points = pointsRes.tryValue()
    var ransac = SeqRansac.init(points, msg.ranges.len, self.params.value.line_fitting)
    let lines = ransac.doSeqRansac()
    self.detectedLines = lines
    self.lastScanTime = initRosTime(msg.header.stamp.sec.int64 * 1_000_000_000 + msg.header.stamp.nanosec.int64)

proc cmdSubLoop(self) {.async.} =
  while true:
    let msg = await self.cmdSub.recv()

    self.turnEnabled = abs(msg.angular.x) > 0.5 and self.params.value.enable_turn
    if msg.angular.x > 0.5:
      self.targetCourse = InCourse
    elif msg.angular.x < -0.5:
      self.targetCourse = OutCourse

    self.angleController.gain = self.params.value.angular_pid_gain

    let enabled = msg.linear.z > 0.5

    if enabled:
      if self.carrotPoint.isSome:
        let speed = vector2f(msg.linear.x, msg.linear.y).length()
        let p = self.carrotPoint.get()
        let line = self.targetLine.get()
        let dir1 = line.direction
        let dir2 = -line.direction
        let dir = 
          if (p - dir1).length2 < (p - dir2).length2:
            dir1
          else:
            dir2
        let pDir = p.normalized()
        let tgtAngle = arctan2(dir.y, dir.x)
        var cmd = Twist()
        self.angleController.target = tgtAngle
        self.angleController.updateWithVel(0.0, self.latestOdom.twist.twist.angular.z, 0.1)
        cmd.linear.x = pDir.x * speed
        cmd.linear.y = pDir.y * speed
        cmd.angular.z = self.angleController.output()
        self.cmdPub.publish cmd
      else:
        var cmd = Twist()
        cmd.angular.z = msg.angular.z
        self.cmdPub.publish cmd
    else:
      var cmd = Twist()
      cmd.linear.x = msg.linear.x
      cmd.linear.y = msg.linear.y
      cmd.angular.z = msg.angular.z
      self.cmdPub.publish msg

proc removeOutdatedOdom(self) =
  let expirationTime = self.node.clock.now() - initDuration(nanoseconds=int64(self.params.value.odom_lifespan * 1e9))
  while self.odomBuffer.len > 0 and self.odomBuffer.peekFirst().stamp < expirationTime:
    discard self.odomBuffer.popFirst()

proc applyOdometry(self; lines: seq[Line], time: RosTime): seq[Line] =
  var
    dx = 0.0
    dy = 0.0
    da = 0.0
  for odom in self.odomBuffer:
    if odom.stamp < time: continue
    dx += odom.dx * cos(da) - odom.dy * sin(da)
    dy += odom.dx * sin(da) + odom.dy * cos(da)
    da += odom.da
  for line in lines:
    let rot = rotationZ(float32 -da)
    let dir = rot * vector3f(line.direction.x, line.direction.y, 0.0)
    let p = rot * (vector3f(line.point.x, line.point.y, 0.0) - vector3f(dx, dy, 0.0))
    result.add Line.fromPointAndDir(vector2f(p.x, p.y), vector2f(dir.x, dir.y))

proc odomSubLoop(self) {.async.} =
  while true:
    let msg = await self.odomSub.recv()
    let msgStamp = initRosTime(msg.header.stamp.sec.int64 * 1_000_000_000 + msg.header.stamp.nanosec.int64)
    let dt = msgStamp - initRosTime(self.latestOdom.header.stamp.sec.int64 * 1_000_000_000 + self.latestOdom.header.stamp.nanosec.int64)
    let dtSec = dt.inNanoseconds.float * 1e-9
    let linear = msg.twist.twist.linear.to(Vector3d)
    let angular = msg.twist.twist.angular.z
    let odom = Odom(dx: linear.x * dtSec, dy: linear.y * dtSec, da: angular * dtSec, stamp: msgStamp)
    self.odomBuffer.addLast odom
    self.removeOutdatedOdom()
    self.latestOdom = msg

    let lines = self.applyOdometry(self.detectedLines, self.lastScanTime)

    let
      linesFiltered = lines.filterIt(self.filterLine(it))
      offsetDistance =
        case self.targetCourse
        of InCourse: self.params.value.in_course_distance
        of OutCourse: self.params.value.out_course_distance
      linesMoved = linesFiltered.mapIt(it.moveParallel(offsetDistance))
      linesIgnored = lines.filterIt(not self.filterLine(it))
      carrotPointAndLine = self.getCarrotPoint(linesMoved)

    if carrotPointAndLine.isSome:
      self.carrotPoint = some carrotPointAndLine.get()[0]
      self.targetLine = some carrotPointAndLine.get()[1]
      self.publishMarkers(linesFiltered, linesIgnored, linesMoved, self.carrotPoint, msg.header.stamp)
    else:
      self.carrotPoint = none Vector2f
      self.targetLine = none Line
      self.publishMarkers(linesFiltered, linesIgnored, linesMoved, none Vector2f, msg.header.stamp)


proc run(self) {.async.} =
  let futures = [
    self.odomSubLoop(),
    self.scanSubLoop(),
    self.cmdSubLoop(),
  ]

  discard await one futures

  for fut in futures:
    if not fut.finished:
      await fut.cancelAndWait()

proc main =
  rclnim.init()
  let wallTracer = newWallTracer()
  waitFor wallTracer.run()

main()
