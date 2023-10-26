#ifndef CAN_BRIDGE_PROTOCOL_H
#define CAN_BRIDGE_PROTOCOL_H

#include <stdint.h>
#include <array>

#define PROTOCOL_PACKED __attribute__((packed))

enum ParamType : uint8_t {
  INT,
  UINT,
  FLOAT
};

struct ParamValue {
  ParamType type;
  union {
    int32_t int_value;
    uint32_t uint_value;
    float float_value;
  } PROTOCOL_PACKED;
} PROTOCOL_PACKED;

// パラメータ
enum ParamId : uint8_t {
  // 速度制御のPIDゲイン
  DRIVE_KP,
  DRIVE_KI,
  DRIVE_KD,
  DRIVE_MAX,
  DRIVE_MIN,
  DRIVE_ANTIWINDUP,
  DRIVE_USE_VELOCITY_FOR_D_TERM,

  // ステア角制御のPIDゲイン
  STEER_KP,
  STEER_KI,
  STEER_KD,
  STEER_MAX,
  STEER_MIN,
  STEER_ANTIWINDUP,
  STEER_USE_VELOCITY_FOR_D_TERM,

  // ステア角の微調整
  STEER0_OFFSET,
  STEER1_OFFSET,
  STEER2_OFFSET,
  STEER3_OFFSET,

    // お助け角制御のPIDゲイン
  ARM_ANGLE_KP,
  ARM_ANGLE_KI,
  ARM_ANGLE_KD,
  ARM_ANGLE_MAX,
  ARM_ANGLE_MIN,
  ARM_ANGLE_ANTIWINDUP,
  ARM_ANGLE_USE_VELOCITY_FOR_D_TERM,

  // お助け長さ制御のPIDゲイン
  ARM_LENGTH_KP,
  ARM_LENGTH_KI,
  ARM_LENGTH_KD,
  ARM_LENGTH_MAX,
  ARM_LENGTH_MIN,
  ARM_LENGTH_ANTIWINDUP,
  ARM_LENGTH_USE_VELOCITY_FOR_D_TERM,

  // ロジャー長さ制御のPIDゲイン
  EXPANDER_KP,
  EXPANDER_KI,
  EXPANDER_KD,
  EXPANDER_MAX,
  EXPANDER_MIN,
  EXPANDER_ANTIWINDUP,
  EXPANDER_USE_VELOCITY_FOR_D_TERM,
};


// PCからマイコンへ送信するメッセージ
struct Command {
  static constexpr int ID = 200;

  enum class Tag : uint8_t {
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
  };

  Tag tag;

  struct GetParam {
      ParamId id;
  } PROTOCOL_PACKED;

  struct SetParam {
    ParamId id;
    ParamValue value;
  } PROTOCOL_PACKED;
  struct SetTargetVelocity {
    int16_t vx; // 前方向の速度 [mm/s]
    int16_t vy; // 左方向の速度 [mm/s]
    int16_t ang_vel; // 上から見て半時計回り方向の角速度 [mrad/s] (ミリラジアン毎秒)
  } PROTOCOL_PACKED;

  struct SetDonfanCmd {
    int8_t dir; // 1: 正転、 0: 停止、 -1: 逆転
  } PROTOCOL_PACKED;

  struct SetExpanderLength {
    int16_t length; // 長さ　下がりきった状態が0 展開方向が+ [mm]
  } PROTOCOL_PACKED;

  struct SetCollectorCmd {
    bool enable; // trueなら回収開始、falseなら回収停止
  } PROTOCOL_PACKED;

  struct SetArmAngle {
    int16_t angle; // 角度 水平が0 前方向が+ [mrad]
  } PROTOCOL_PACKED;

  struct SetArmLength {
    int16_t length; // 長さ 下がりきった状態が0 展開方向が+ [mm]
  } PROTOCOL_PACKED;

  struct SetLargeWheelCmd {
    int16_t cmd; // +正転 -逆転 32767が最大
  } PROTOCOL_PACKED;

  struct WallAlignAssist {
    uint16_t distance; // 目標壁との距離 [mm] 0のとき無効
  } PROTOCOL_PACKED;

  union {
    // パラメータを取得する。
    GetParam get_param;

    // パラメータを設定する。
    // 設定するパラメータがPIDのゲインの場合は、ResetPidが送られるまで適用しない。
    SetParam set_param;

    // 目標速度を設定する
    SetTargetVelocity set_target_velocity;

    // ドンファン
    SetDonfanCmd set_donfan_cmd;

    // ロジャー
    SetExpanderLength set_expander_length;

    // 下から回収
    SetCollectorCmd set_collector_cmd;

    // お助け角度
    SetArmAngle set_arm_angle;

    // お助け長さ
    SetArmLength set_arm_length;

    //　段超え
    SetLargeWheelCmd set_large_wheel_cmd;

    // 壁との距離を一定に保つ
    WallAlignAssist wall_align_assist;
  };
} PROTOCOL_PACKED;


// マイコンからPCへ送信するメッセージ
struct Feedback {
  static constexpr int ID = 201;

  enum class Tag : uint8_t {
    PARAM_EVENT,
    GET_PARAM_RESPONSE,
    VELOCITY,
    POSE,
    HEARTBEAT,
    STEER_UNWIND_DONE,
    CURRENT_STATE,
    STEER_UNIT_STATE,
    DETECTED_WALL,
  };

  Tag tag;

  struct ParamEvent {
    ParamId id;
    ParamValue value;
  } PROTOCOL_PACKED;

  struct GetParamResponse {
    ParamId id;
    ParamValue value;
  } PROTOCOL_PACKED;

  struct Velocity {
    int16_t vx; // 前方向の速度 [mm/s]
    int16_t vy; // 左方向の速度 [mm/s]
    int16_t ang_vel; // 上から見て半時計回り方向の角速度 [mrad/s] (ミリラジアン毎秒)
  } PROTOCOL_PACKED;

  struct Pose {
    uint16_t x; // x座標 [mm]
    uint16_t y; // y座標 [mm]
    uint16_t yaw; // 向き [mrad]
  } PROTOCOL_PACKED;

  struct CurrentState {
    enum State {
      CONFIGURING,
      RUNNING,
    } state;
  } PROTOCOL_PACKED;

  struct SteerUnitState {
    uint8_t index; // ステアの番号 0:左前 1:左後ろ 2:右後ろ 3:右前
    int16_t velocity; // 速度　[mrad/s]
    int16_t current; // 電流 [mA]
    int16_t angle; // ステア角 0~2π [mrad]
  } PROTOCOL_PACKED;

  struct DetectedWall {
    uint16_t distance; // 壁との距離 [mm]
    int16_t angle; // 壁の角度 0のときまっすぐ　[mrad]
  } PROTOCOL_PACKED;

  union {
    // パラメータのイベント
    // パラメータが設定されたときに送る
    ParamEvent param_event;

    // `get_param`に対する返信
    GetParamResponse get_param_response;

    // 速度
    Velocity odometry;

    // 位置・姿勢
    Pose position;

    CurrentState current_state;

    SteerUnitState steer_unit_state;

    // 検出された壁の情報
    DetectedWall detected_wall;
  };
} PROTOCOL_PACKED;

static_assert(sizeof(Command) <= 8);
static_assert(sizeof(Feedback) <= 8);

#endif
