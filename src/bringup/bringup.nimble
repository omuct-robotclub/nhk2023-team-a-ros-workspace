# Package

version       = "0.1.0"
author        = "Pylgos"
description   = "A new awesome nimble package"
license       = "MIT"

# Dependencies

requires "nim >= 1.9.5"

const installBase {.define.} = ""

task postInstall, "post install task":
  cpDir "launch", installBase & "/share/bringup/launch"
  cpDir "config", installBase & "/share/bringup/config"
  cpDir "urdf", installBase & "/share/bringup/urdf"
  cpDir "map", installBase & "/share/bringup/map"
