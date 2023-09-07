import std/heapqueue
import std/math


type
  Grid*[T] = object
    data*: seq[T]
    width*, height*: int

  DijkstraGrid* = object
    map*: Grid[uint8]
    costs*: Grid[float]

proc `[]`*[T](grid: var Grid[T], x, y: int): var T =
  assert 0 <= x and x < grid.width
  assert 0 <= y and y < grid.height
  grid.data[x + y * grid.width]

proc `[]`*[T](grid: Grid[T], x, y: int): T =
  assert 0 <= x and x < grid.width
  assert 0 <= y and y < grid.height
  grid.data[x + y * grid.width]

proc fill*[T](grid: var Grid[T], value: T) =
  for d in grid.data.mitems:
    d = value

proc resize*[T](grid: var Grid[T], width, height: int, value: T) =
  grid.data = newSeq[T](width * height, value)
  grid.width = width
  grid.height = height

proc contains*[T](grid: Grid[T], pos: (int, int)): bool =
  0 <= pos[0] and pos[0] < grid.width and 0 <= pos[1] and pos[1] < grid.height

iterator neighbors*[T](grid: Grid[T], at: (int, int)): (float, (int, int)) =
  const r2 = sqrt(2'f64)
  const offsets = [
    (-1, -1, r2), ( 0, -1, 1), ( 1, -1, r2),
    (-1,  0,  1),              ( 1,  0,  1),
    (-1,  1, r2), ( 0,  1, 1), ( 1,  1, r2),
  ]
  for offset in offsets:
    let p = (offset[0] + at[0], offset[1] + at[1])
    if p in grid:
      yield (offset[2], p)

using self: var DijkstraGrid

proc init*(_: typedesc[DijkstraGrid]): DijkstraGrid =
  discard

proc setMap*(self; data: seq[uint8], width, height: int) =
  self.map = Grid[uint8](data: data, width: width, height: height)

type E = object
  p: (int, int)
  c: float

proc `<`(a, b: E): bool =
  a.c < b.c

proc dijkstra*(self; start, goal: (int, int)) =
  var q = initHeapQueue[E]()
  
  # for v in self.map.neighbors(start):
  #   q.push E(c: v[0], p: v[1])
  
  # while q.len > 0:
  #   let u = q.pop()
  #   for v in self.map.neighbors(u.p):


