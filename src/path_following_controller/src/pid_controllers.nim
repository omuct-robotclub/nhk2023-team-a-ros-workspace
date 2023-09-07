import std/math

type
  PidGain* = object
    kp*, ki*, kd*: float
    max*, min*: float
    useVelocityForDTerm*: bool
    antiWindup*: bool
  
  PidController* = object
    gain*: PidGain
    target*: float
    output: float
    prev: float
    integral: float
    prevError: float


proc initPidController*(gain: PidGain): PidController =
  result.gain = gain
  result.output = 0.0
  result.target = NaN
  result.prev = NaN
  result.integral = 0.0
  result.prevError = 0.0

proc init*(_: typedesc[PidController], gain: PidGain): PidController =
  initPidController(gain)

proc updateWithVel*(self: var PidController, present: float, velocity: float, dt: float) =
  self.prev = present
  if (isNaN(self.target) or isNaN(present) or isNaN(velocity) or dt == 0.0):
    return
  let error = self.target - present
  let p = error * self.gain.kp
  self.integral += error * dt
  let i = self.integral * self.gain.ki
  let d =
    if self.gain.useVelocityForDTerm:
      velocity * self.gain.kd
    else:
      (error - self.prevError) / dt * self.gain.kd
  self.prevError = error
  let output = p + i + d
  let outputSaturated = clamp(output, self.gain.min..self.gain.max)
  if self.gain.antiWindup and self.gain.ki != 0:
    let integralMax = self.gain.max / self.gain.ki;
    let integralMin = self.gain.min / self.gain.ki;
    self.integral = clamp(self.integral, integralMin..integralMax)
  self.output = outputSaturated

proc update*(self: var PidController, present: float, dt: float) =
  let vel =
    if isNaN(self.prev):
      0.0
    else:
      (present - self.prev) / dt
  self.updateWithVel(present, vel, dt)

proc output*(self: PidController): float = self.output

proc reset*(self: var PidController) =
  self.output = 0
  self.target = NaN
  self.prev = NaN
  self.integral = 0
  self.prevError = 0
