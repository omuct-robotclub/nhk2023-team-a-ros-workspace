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
    SET_EXPANDER_CMD,
    SET_COLLECTOR_CMD,
    SET_ARM_ANGLE,
    SET_ARM_LENGTH,
    UNWIND_STEER,
    SET_LARGE_WHEEL_CMD,
    ACTIVATE,
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

  struct SetExpanderCmd {
    int16_t cmd; // +展開
  } PROTOCOL_PACKED;

  struct SetCollectorCmd {
    bool enable; // trueなら回収開始、falseなら回収停止
  } PROTOCOL_PACKED;

  struct SetArmAngle {
    int16_t angle; // 角度 初期位置が0 前方向が+ [mrad]
  } PROTOCOL_PACKED;

  struct SetArmLength {
    int16_t length; // 長さ 初期位置が0 展開方向が+ [mm]
  } PROTOCOL_PACKED;

  struct SetLargeWheelCmd {
    int16_t cmd; // +正転 -逆転 32767が最大
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
    SetExpanderCmd set_expander_cmd;

    // 下から回収
    SetCollectorCmd set_collector_cmd;

    // お助け角度
    SetArmAngle set_arm_angle;

    // お助け長さ
    SetArmLength set_arm_length;

    //　段超え
    SetLargeWheelCmd set_large_wheel_cmd;
  };
} PROTOCOL_PACKED;


// マイコンからPCへ送信するメッセージ
struct Feedback {
  static constexpr int ID = 201;

  enum class Tag : uint8_t {
    PARAM_EVENT,
    GET_PARAM_RESPONSE,
    ODOMETRY,
    HEARTBEAT,
    STEER_UNWIND_DONE,
    CURRENT_STATE,
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

  struct Odometry {
    int16_t vx; // 前方向の速度 [mm/s]
    int16_t vy; // 左方向の速度 [mm/s]
    int16_t ang_vel; // 上から見て半時計回り方向の角速度 [mrad/s] (ミリラジアン毎秒)
  } PROTOCOL_PACKED;

  struct CurrentState {
    enum State {
      CONFIGURING,
      RUNNING,
    } state;
  } PROTOCOL_PACKED;

  union {
    // パラメータのイベント
    // パラメータが設定されたときに送る
    ParamEvent param_event;

    // `get_param`に対する返信
    GetParamResponse get_param_response;

    // オドメトリの情報
    Odometry odometry;

    CurrentState current_state;
  };
} PROTOCOL_PACKED;

static_assert(sizeof(Command) <= 8);
static_assert(sizeof(Feedback) <= 8);

#endif
