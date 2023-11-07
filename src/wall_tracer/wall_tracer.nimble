# Package

version       = "0.1.0"
author        = "Pylgos"
description   = "A new awesome nimble package"
license       = "MIT"
srcDir        = "src"
bin           = @["wall_tracer"]


# Dependencies

requires "nim >= 1.9.5"
requires "https://github.com/pylgos/rclnim.git#head"
requires "https://github.com/Pylgos/geonimetry.git"
requires "https://github.com/Pylgos/nim-tf2.git"
requires "https://github.com/Pylgos/toconv.git"

