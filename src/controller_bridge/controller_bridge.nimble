# Package

version       = "0.1.0"
author        = "Pylgos"
description   = "A new awesome nimble package"
license       = "MIT"
srcDir        = "src"
bin           = @["controller_bridge"]


# Dependencies

requires "nim >= 2.0"
requires "https://github.com/pylgos/rclnim.git#head"
requires "vmath"
requires "https://github.com/pylgos/nim-socketcan.git"
requires "websock"
