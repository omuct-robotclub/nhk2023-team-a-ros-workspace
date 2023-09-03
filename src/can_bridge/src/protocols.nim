

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
    DRIVE_KP,
    DRIVE_KI,
    DRIVE_KD,
    DRIVE_MAX,
    DRIVE_MIN,
    DRIVE_ANTIWINDUP,
    DRIVE_USE_VELOCITY_FOR_D_TERM,

    STEER_KP,
    STEER_KI,
    STEER_KD,
    STEER_MAX,
    STEER_MIN,
    STEER_ANTIWINDUP,
    STEER_USE_VELOCITY_FOR_D_TERM,

    STEER0_OFFSET,
    STEER1_OFFSET,
    STEER2_OFFSET,
    STEER3_OFFSET,


  # commands
  RoboCmdKind* {.size: 1.} = enum
    GET_PARAM,
    SET_PARAM,
    RESET_PID,
    SET_TARGET_VELOCITY,
    SET_DONFAN_CMD,
    SET_EXPANDER_CMD,
    SET_COLLECTOR_CMD,
    SET_ARM_ANGLE,
    SET_ARM_LENGTH,
    UNWIND_STEER,
    SET_LARGE_WHEEL_CMD,
    ACTIVATE,

  GetParamObj* {.packed.} = object
    id*: RoboParamId

  SetParamObj* {.packed.} = object
    id*: RoboParamId
    value*: RoboParamValue
  SetTargetVelocityObj* {.packed.} = object
    vx*, vy*, angVel*: int16
  
  SetDonfanCmdObj* {.packed.} = object
    dir*: int8
  
  SetExpanderCmdObj* {.packed.} = object
    cmd*: int16
  
  SetCollectorCmdObj* {.packed.} = object
    cmd*: int16
  
  SetArmAngleObj* {.packed.} = object
    angle*: int16
  
  SetArmLengthObj* {.packed.} = object
    length*: int16
  
  SetLargeWheelCmdObj* {.packed.} = object
    cmd*: int16

  RoboCmd* {.packed.} = object
    case kind*: RoboCmdKind
    of GET_PARAM: getParam*: GetParamObj
    of SET_PARAM: setParam*: SetParamObj
    of RESET_PID: discard
    of SET_TARGET_VELOCITY: setTargetVelocity*: SetTargetVelocityObj
    of SET_DONFAN_CMD: setDonfanCmd*: SetDonfanCmdObj
    of SET_EXPANDER_CMD: setExpanderCmd*: SetExpanderCmdObj
    of SET_COLLECTOR_CMD: setCollectorCmd*: SetCollectorCmdObj
    of SET_ARM_ANGLE: setArmAngle*: SetArmAngleObj
    of SET_ARM_LENGTH: setArmLength*: SetArmLengthObj
    of SET_LARGE_WHEEL_CMD: setLargeWheelCmd*: SetLargeWheelCmdObj
    of UNWIND_STEER: discard
    of ACTIVATE: discard

  RoboCmdData* {.union.} = object
    bytes*: array[8, byte]
    msg*: RoboCmd

  # feedbacks
  RoboFeedbackKind* {.size: 1.} = enum
    PARAM_EVENT,
    GET_PARAM_RESPONSE,
    ODOMETRY,
    HEARTBEAT,
    STEER_UNWIND_DONE,
    CURRENT_STATE

  ParamEventObj* {.packed.} = object
    id*: RoboParamId
    value*: RoboParamValue

  GetParamResponseObj* {.packed.} = object
    id*: RoboParamId
    value*: RoboParamValue

  OdometryObj* {.packed.} = object
    vx*, vy*, angVel*: int16

  CurrentStateEnum* {.size: 1.} = enum
    Configuring
    Running

  CurrentStateObj* {.packed.} = object
    state*: CurrentStateEnum

  RoboFeedback* {.packed.} = object
    case kind*: RoboFeedbackKind
    of PARAM_EVENT: paramEvent*: ParamEventObj
    of GET_PARAM_RESPONSE: getParamResponse*: GetParamResponseObj
    of ODOMETRY: odometry*: OdometryObj
    of HEARTBEAT: discard
    of STEER_UNWIND_DONE: discard
    of CURRENT_STATE: currentState*: CurrentStateObj

  RoboFeedbackData* {.union.} = object
    bytes*: array[8, byte]
    msg*: RoboFeedback


static:
  doAssert sizeof(RoboParamValue) == 5
  doAssert sizeof(RoboCmdData) == 8
  doAssert sizeof(RoboFeedbackData) == 8
