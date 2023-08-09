# Package

version       = "0.1.0"
author        = "Pylgos"
description   = "A new awesome nimble package"
license       = "MIT"
srcDir        = "src"
bin           = @["can_bridge"]


# Dependencies

requires "nim >= 1.9.5"
requires "https://github.com/pylgos/rclnim.git#head"
requires "vmath"
requires "https://github.com/pylgos/nim-socketcan.git"


task setupVcan, "setup virtual can interface":
  exec "sudo modprobe vcan"
  exec "sudo ip link add dev vcan0 type vcan"
  exec "sudo ip link set vcan0 up"

task teardownVcan, "teardon virtual can interface":
  exec "sudo ip link del dev vcan0"
