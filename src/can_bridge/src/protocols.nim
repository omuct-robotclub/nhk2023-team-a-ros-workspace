

type
  # parameter things
  RoboParamKind* {.size: 1.} = enum
    Int
    UInt
    Float

  RoboParamValue* {.packed.} = object
    case kind*: RoboParamKind
    of Int:
      intVal*: int32
    of UInt:
      uintVal*: uint32
    of Float:
      floatVal*: float32

  RoboParamId* {.size: 1.} = enum
    X_Kp
    X_Ki
    X_Kd

    Y_Kp
    Y_Ki
    Y_Kd

    Yaw_Kp
    Yaw_Ki
    Yaw_Kd


  # commands
  RoboCmdKind* {.size: 1.} = enum
    GetParam
    SetParam
    ResetPid
    SetTargetVelocity
    GetOver
    SetDonfanCmd
    SetExpanderCmd
    SetCollectorCmd
    SetArmCmd

  GetParamObj* {.packed.} = object
    id*: RoboParamId

  SetParamObj* {.packed.} = object
    id*: RoboParamId
    value*: RoboParamValue

  ResetPidObj* {.packed.} = object

  SetTargetVelocityObj* {.packed.} = object
    vx*, vy*, angVel*: int16

  StepKind* {.size: 1.} = enum
    Small
    Large
    Center

  GetOverObj* {.packed.} = object
    stepKind*: StepKind
  
  SetDonfanCmdObj* {.packed.} = object
    cmd*: int16
  
  SetExpanderCmdObj* {.packed.} = object
    cmd*: int16
  
  SetCollectorCmdObj* {.packed.} = object
    cmd*: int16
  
  SetArmCmdObj* {.packed.} = object
    index*: uint8
    expanderCmd*: int16
    tiltCmd*: int16

  RoboCmd* {.packed.} = object
    case kind*: RoboCmdKind
    of GetParam: getParam*: GetParamObj
    of SetParam: setParam*: SetParamObj
    of ResetPid: resetPid*: ResetPidObj
    of SetTargetVelocity: setTargetVelocity*: SetTargetVelocityObj
    of GetOver: getOver*: GetOverObj
    of SetDonfanCmd: setDonfanCmd*: SetDonfanCmdObj
    of SetExpanderCmd: setExpanderCmd*: SetExpanderCmdObj
    of SetCollectorCmd: setCollectorCmd*: SetCollectorCmdObj
    of SetArmCmd: setArmCmd*: SetArmCmdObj

  RoboCmdData* {.union.} = object
    bytes*: array[8, byte]
    msg*: RoboCmd


  # feedbacks
  RoboFeedbackKind* {.size: 1.} = enum
    ParamEvent
    GetParamResponse
    Odometry
    GetOverDone
    Heartbeat

  ParamEventObj* {.packed.} = object
    id*: RoboParamId
    value*: RoboParamValue

  GetParamResponseObj* {.packed.} = object
    id*: RoboParamId
    value*: RoboParamValue

  OdometryObj* {.packed.} = object
    vx*, vy*, angVel*: int16

  GetOverDoneObj* {.packed.} = object

  HeartbeatObj* {.packed.} = object

  RoboFeedback* {.packed.} = object
    case kind*: RoboFeedbackKind
    of ParamEvent:
      paramEvent*: ParamEventObj
    of GetParamResponse:
      getParamResponse*: GetParamResponseObj
    of Odometry:
      odometry*: OdometryObj
    of GetOverDone:
      getOverDone*: GetOverDoneObj
    of Heartbeat:
      heartbeat*: HeartbeatObj

  RoboFeedbackData* {.union.} = object
    bytes*: array[8, byte]
    msg*: RoboFeedback


static:
  doAssert sizeof(RoboParamValue) == 5
  doAssert sizeof(RoboCmdData) == 8
  doAssert sizeof(RoboFeedbackData) == 8
