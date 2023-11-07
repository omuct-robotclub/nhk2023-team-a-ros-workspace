import geonimetry
import std/[math, options, random]

type
  Line* = object
    point*: Vector2f
    direction*: Vector2f

{.push inline.}

proc fromPointAndDir*(_: typedesc[Line], p, d: Vector2f): Line =
  Line(point: p, direction: d.normalized())

proc from2Points*(_: typedesc[Line], p1, p2: Vector2f): Line =
  Line(point: p1, direction: (p2 - p1).normalized())

proc distanceToPoint*(l: Line, p: Vector2f): float =
  abs(l.direction.x * (l.point.y - p.y) - (l.point.x - p.x) * l.direction.y)

proc intersection*(a, b: Line): Option[Vector2f] =
  let denom = b.direction.y * a.direction.x - b.direction.x * a.direction.y
  if abs(denom) < 1e-9: return none Vector2f
  let v = a.point - b.point
  let t = (b.direction.x * v.y - b.direction.y * v.x) / denom
  return some a.point + a.direction * t

proc getClosestPointTo*(point: Vector2f, line: Line): Vector2f =
  let p = point - line.point
  let n = line.direction
  let d = n.dot(p)
  line.point + n * d

proc intersectionToCircle*(l: Line, center: Vector2f, radius: float32): Option[array[2, Vector2f]] =
  let
    a = l.direction.x^2 + l.direction.y^2
    b = 2 * (l.direction.x * (l.point.x - center.x) + l.direction.y * (l.point.y - center.y))
    c =  (l.point.x - center.x)^2 + (l.point.y - center.y)^2 - radius^2
  
    discriminant = b^2 - 4 * a * c
  
  if discriminant >= 0:
    let
      t1 = (-b + sqrt(discriminant)) / (2 * a)
      t2 = (-b - sqrt(discriminant)) / (2 * a)
      p1 = l.point + l.direction * t1
      p2 = l.point + l.direction * t2
    some [p1, p2]
  else:
    none array[2, Vector2f]

{.pop.}

type
  SeqRansacConfig* = object
    inlierDistanceThreshold*: float
    iterationCount*: int
    minInlierRatio*: float
    minPointsRatio*: float
    minDistanceBetweenSamples*: float

  SeqRansac* = object
    config: SeqRansacConfig
    rng: Rand
    originalPointCount: int
    points: seq[Vector2f]

{.push inline.}

proc init*(_: typedesc[SeqRansac], points: sink seq[Vector2f], config = SeqRansacConfig()): SeqRansac =
  SeqRansac(config: config, originalPointCount: points.len, points: points, rng: initRand())

proc pickTwoPoints(self: var SeqRansac): (Vector2f, Vector2f) {.inline.} =
  var
    i0 = 0
    i1 = 0
  while i0 == i1:
    i0 = self.rng.rand(0..self.points.high)
    i1 = self.rng.rand(0..self.points.high)
  (self.points[i0], self.points[i1])

proc doRansac(self: var SeqRansac): Option[Line] =
  if self.points.len / self.originalPointCount < self.config.minPointsRatio or self.points.len < 2:
    return none Line

  var bestLine: Line
  var bestInliers: seq[Vector2f]
  var bestOutliers: seq[Vector2f]
  var bestLoss = Inf
  
  for i in 0..<self.config.iterationCount:
    let (p1, p2) = self.pickTwoPoints()
    if (p2 - p1).length < self.config.minDistanceBetweenSamples:
      continue
    let line = Line.from2Points(p1, p2)
    var inliers: seq[Vector2f]
    var outliers: seq[Vector2f]
    var loss = 0.0
    
    for p in self.points:
      let dist = line.distanceToPoint(p)
      let isInlier = dist < self.config.inlierDistanceThreshold
      if isInlier:
        loss += dist^2 / self.originalPointCount.float
        inliers.add p
      else:
        outliers.add p
    
    if loss < bestLoss and
        inliers.len / self.originalPointCount > self.config.minInlierRatio:
      bestInliers = inliers
      bestOutliers = outliers
      bestLoss = loss
      bestLine = line

  if bestInliers.len == 0:
    none Line
  else:
    self.points = bestOutliers
    some bestLine

proc doSeqRansac*(self: var SeqRansac): seq[Line] =
  while true:
    let res = self.doRansac()
    if res.isSome:
      result.add res.get()
    else:
      return

{.pop.}

when isMainModule:
  block:
    let axisX = Line.fromPointAndDir(vector2f(-1, 0), vector2f(1, 0))
    let axisY = Line.fromPointAndDir(vector2f(0, 0), vector2f(0, 1))
    let p1 = vector2f(0, 1)
    let p2 = vector2f(2, 0)
    
    doAssert axisX.distanceToPoint(p1) == 1.0
    doAssert axisY.distanceToPoint(p1) == 0.0

    doAssert axisX.distanceToPoint(p2) == 0.0
    doAssert axisY.distanceToPoint(p2) == 2.0

    let l1 = Line.fromPointAndDir(vector2f(-1, -1), vector2f(1, 1))
    echo l1.distanceToPoint(p2)

    echo p1.getClosestPointTo(l1)
    echo axisX.intersectionToCircle(vector2f(0, 4), 3)
