import rclnim
import rclnim/chronossupport
import tf2
import tf2/msgconverters
import chronos
import geonimetry
import ./linefitting
import std/[options, sequtils, math, times, algorithm]
import results

importInterface std_msgs/msg/color_rgba
importInterface sensor_msgs/msg/laser_scan
importInterface visualization_msgs/msg/[marker, marker_array]
importInterface nav_msgs/msg/odometry
importInterface geometry_msgs/msg/[point, twist]
importInterface builtin_interfaces/msg/time as time_msg, Time as TimeMsg

type
  Parameters = object
    enable_wall_tracing: bool
    robot_size_x: float
    robot_size_y: float
    lookahead_distance: float
    extra_lookahead_distance_per_velocity: float
    in_course_distance: float
    out_course_distance: float
    line_fitting: SeqRansacConfig

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

    targetCurvature: float
    targetCourse: Course
    turnEnabled: bool

  Course = enum
    InCourse
    OutCourse

using self: WallTracer

proc newWallTracer(): WallTracer =
  result = new WallTracer
  result.node = newNode("wall_tracer")
  result.buf = TfBuffer.new(result.node)
  result.params = result.node.createObjParamServer(Parameters(
    robot_size_x: 0.7,
    robot_size_y: 0.7,
    lookahead_distance: 0.75,
    extra_lookahead_distance_per_velocity: 0.5,
    in_course_distance: 2.25,
    out_course_distance: 0.75,
    line_fitting: SeqRansacConfig(
      inlierDistanceThreshold: 0.05,
      minInlierRatio: 0.2,
      iterationCount: 2000,
      minPointsRatio: 0.2,
      minDistanceBetweenSamples: 1.0,
    )
  ))
  result.scanSub = result.node.createSubscription(LaserScan, "scan", SensorDataQoS)
  result.cmdSub = result.node.createSubscription(Twist, "manual_cmd_vel", SystemDefaultQoS)
  result.cmdPub = result.node.createPublisher(Twist, "cmd_vel", SystemDefaultQoS)
  result.markerPub = result.node.createPublisher(MarkerArray, "wall_tracer_markers", SystemDefaultQoS)
  result.odomSub = result.node.createSubscription(Odometry, "odom", SensorDataQoS)
  result.targetCurvature = NaN
  result.targetCourse = OutCourse

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

proc publishMarkers(self; filtered, ignored, moved: seq[Line], carrotPoint: Option[Vector2f], curvature: Option[float], stamp: TimeMsg) =
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
  
  if curvature.isSome:
    let curvature = curvature.get()
    var m = Marker()
    m.header.frameId = "base_link"
    m.header.stamp = stamp
    m.id = int32 markers.markers.len
    m.`type` = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.05
    m.scale.y = 0.05
    m.scale.z = 0.05
    m.color = carrotPointColor
    m.lifetime.nanosec = 200_000_000
    var
      x = 0'f
      y = 0'f
      a = 0'f
    let dt = 0.01
    let vx = 1.0
    for i in 0..<100:
      x += vx * cos(a) * dt
      y += vx * sin(a) * dt
      a += vx * curvature * dt
      m.points.add Point(x: x, y: y, z: 0.0)
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

proc getCarrotPoint(self; lines: openArray[Line]): Option[Vector2f] =
  var candidates = newSeq[Vector2f]()
  let presentSpeed = self.latestOdom.twist.twist.linear.to(Vector3d).length()
  var lookaheadDist = self.params.value.lookahead_distance + self.params.value.extra_lookahead_distance_per_velocity * presentSpeed
  for i in 0..<100:
    lookaheadDist += 0.1
    for l in lines:
      let res = l.intersectionToCircle(vector2f(0, 0), lookaheadDist)
      if res.isNone: continue
      let points = res.get().filterIt(it.x > 0)
      if points.len == 0: continue
      candidates.add points
    
    if candidates.len > 0:
      return some candidates.sortedByIt(arctan2(it.y, it.x))[^1]
  none Vector2f

proc scanSubLoop(self) {.async.} =
  while true:
    let msg = await self.scanSub.recv()
    let pointsRes = self.transformScanToPoints(msg)
    if pointsRes.isErr:
      self.node.getLogger.warn "failed to transform laser scan"
      continue
    let points = pointsRes.tryValue()
    var ransac = SeqRansac.init(points, self.params.value.line_fitting)
    
    let
      lines = ransac.doSeqRansac()
      linesFiltered = lines.filterIt(self.filterLine(it))
      offsetDistance =
        case self.targetCourse
        of InCourse: self.params.value.in_course_distance
        of OutCourse: self.params.value.out_course_distance
      linesMoved = linesFiltered.mapIt(it.moveParallel(offsetDistance))
      linesIgnored = lines.filterIt(not self.filterLine(it))
      carrotPoint = self.getCarrotPoint(linesMoved)

    if carrotPoint.isSome:
      let carrotPoint = carrotPoint.get()
      let turnAngle = arctan2(carrotPoint.y, carrotPoint.x)
      let curvature = 2 * sin(turnAngle) / carrotPoint.length()
      self.targetCurvature = curvature
      self.publishMarkers(linesFiltered, linesIgnored, linesMoved, some carrotPoint, some curvature.float64, msg.header.stamp)
    else:
      self.targetCurvature = NaN
      self.publishMarkers(linesFiltered, linesIgnored, linesMoved, none Vector2f, none float, msg.header.stamp)

proc cmdSubLoop(self) {.async.} =
  while true:
    let msg = await self.cmdSub.recv()
    self.turnEnabled = abs(msg.angular.x) > 0.5
    if msg.angular.x > 0.5:
      self.targetCourse = InCourse
    elif msg.angular.x < -0.5:
      self.targetCourse = OutCourse

    let enabled = msg.linear.z > 0.5

    if enabled:
      if not self.targetCurvature.isNaN:
        let linear = msg.linear.to(Vector3d)
        var cmd = Twist()
        cmd.linear.x = linear.x
        cmd.angular.z = 
          if abs(linear.x * self.targetCurvature) > abs(msg.angular.z):
            linear.x * self.targetCurvature
          else:
            msg.angular.z
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

proc odomSubLoop(self) {.async.} =
  while true:
    let msg = await self.odomSub.recv()
    self.latestOdom = msg

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
