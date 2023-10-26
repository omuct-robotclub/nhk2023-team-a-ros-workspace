

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

    ARM_ANGLE_KP,
    ARM_ANGLE_KI,
    ARM_ANGLE_KD,
    ARM_ANGLE_MAX,
    ARM_ANGLE_MIN,
    ARM_ANGLE_ANTIWINDUP,
    ARM_ANGLE_USE_VELOCITY_FOR_D_TERM,

    ARM_LENGTH_KP,
    ARM_LENGTH_KI,
    ARM_LENGTH_KD,
    ARM_LENGTH_MAX,
    ARM_LENGTH_MIN,
    ARM_LENGTH_ANTIWINDUP,
    ARM_LENGTH_USE_VELOCITY_FOR_D_TERM,

    EXPANDER_KP,
    EXPANDER_KI,
    EXPANDER_KD,
    EXPANDER_MAX,
    EXPANDER_MIN,
    EXPANDER_ANTIWINDUP,
    EXPANDER_USE_VELOCITY_FOR_D_TERM,


  # commands
  RoboCmdKind* {.size: 1.} = enum
    GET_PARAM,
    SET_PARAM,
    RESET_PID,
    SET_TARGET_VELOCITY,
    SET_DONFAN_CMD,
    SET_EXPANDER_LENGTH,
    SET_COLLECTOR_CMD,
    SET_ARM_ANGLE,
    SET_ARM_LENGTH,
    UNWIND_STEER,
    SET_LARGE_WHEEL_CMD,
    ACTIVATE,
    WALL_ALIGN_ASSIST,

  GetParamObj* {.packed.} = object
    id*: RoboParamId

  SetParamObj* {.packed.} = object
    id*: RoboParamId
    value*: RoboParamValue
  SetTargetVelocityObj* {.packed.} = object
    vx*, vy*, angVel*: int16
  
  SetDonfanCmdObj* {.packed.} = object
    dir*: int8
  
  SetExpanderLengthObj* {.packed.} = object
    length*: int16
  
  SetCollectorCmdObj* {.packed.} = object
    enable*: bool
  
  SetArmAngleObj* {.packed.} = object
    angle*: int16
  
  SetArmLengthObj* {.packed.} = object
    length*: int16
  
  SetLargeWheelCmdObj* {.packed.} = object
    cmd*: int16
  
  WallAlignAssistObj* {.packed.} = object
    distance: uint16

  RoboCmd* {.packed.} = object
    case kind*: RoboCmdKind
    of GET_PARAM: getParam*: GetParamObj
    of SET_PARAM: setParam*: SetParamObj
    of RESET_PID: discard
    of SET_TARGET_VELOCITY: setTargetVelocity*: SetTargetVelocityObj
    of SET_DONFAN_CMD: setDonfanCmd*: SetDonfanCmdObj
    of SET_EXPANDER_LENGTH: setExpanderLength*: SetExpanderLengthObj
    of SET_COLLECTOR_CMD: setCollectorCmd*: SetCollectorCmdObj
    of SET_ARM_ANGLE: setArmAngle*: SetArmAngleObj
    of SET_ARM_LENGTH: setArmLength*: SetArmLengthObj
    of SET_LARGE_WHEEL_CMD: setLargeWheelCmd*: SetLargeWheelCmdObj
    of UNWIND_STEER: discard
    of ACTIVATE: discard
    of WALL_ALIGN_ASSIST: wallAlignAssist*: WallAlignAssistObj

  RoboCmdData* {.union.} = object
    bytes*: array[8, byte]
    msg*: RoboCmd

  # feedbacks
  RoboFeedbackKind* {.size: 1.} = enum
    PARAM_EVENT,
    GET_PARAM_RESPONSE,
    VELOCITY,
    POSE,
    HEARTBEAT,
    STEER_UNWIND_DONE,
    CURRENT_STATE,
    STEER_UNIT_STATE,
    DETECTED_WALL,

  ParamEventObj* {.packed.} = object
    id*: RoboParamId
    value*: RoboParamValue

  GetParamResponseObj* {.packed.} = object
    id*: RoboParamId
    value*: RoboParamValue

  VelocityObj* {.packed.} = object
    vx*, vy*, angVel*: int16
  
  PoseObj* {.packed.} = object
    x*, y*, yaw*: uint16

  CurrentStateEnum* {.size: 1.} = enum
    Configuring
    Running

  CurrentStateObj* {.packed.} = object
    state*: CurrentStateEnum
  
  SteerUnitStateObj* {.packed.} = object
    index*: uint8
    velocity*, current*, angle*: int16
  
  DetectedWallObj* {.packed.} = object
    distance: uint16
    angle: int16

  RoboFeedback* {.packed.} = object
    case kind*: RoboFeedbackKind
    of PARAM_EVENT: paramEvent*: ParamEventObj
    of GET_PARAM_RESPONSE: getParamResponse*: GetParamResponseObj
    of VELOCITY: velocity*: VelocityObj
    of POSE: pose*: PoseObj
    of HEARTBEAT: discard
    of STEER_UNWIND_DONE: discard
    of CURRENT_STATE: currentState*: CurrentStateObj
    of STEER_UNIT_STATE: steerUnitState*: SteerUnitStateObj
    of DETECTED_WALL: detectedWall*: DetectedWallObj

  RoboFeedbackData* {.union.} = object
    bytes*: array[8, byte]
    msg*: RoboFeedback


static:
  doAssert sizeof(RoboParamValue) == 5
  doAssert sizeof(RoboCmdData) == 8
  doAssert sizeof(RoboFeedbackData) == 8
