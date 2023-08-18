#ifndef CAN_BRIDGE_PROTOCOL_H
#define CAN_BRIDGE_PROTOCOL_H

#include <stdint.h>
#include <array>

#define PACKED __attribute__((packed))

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
  } PACKED;
} PACKED;

// パラメータ
enum ParamId : uint8_t {
  // 速度制御のPIDゲイン
  KP_X, // float
  KI_X, // float
  KD_X, // float

  KP_Y, // float
  KI_Y, // float
  KD_Y, // float

  KP_YAW, // float
  KI_YAW, // float
  KD_YAW, // float
};


// PCからマイコンへ送信するメッセージ
struct Command {
  static constexpr int ID = 200;

  enum class Tag : uint8_t {
    GET_PARAM,
    SET_PARAM,
    RESET_PID,
    SET_TARGET_VELOCITY,
    GET_OVER,
    SET_DONFAN_CMD,
    SET_EXPANDER_CMD,
    SET_COLLECTOR_CMD,
    SET_ARM_CMD,
  };

  Tag tag;

  union {
    // パラメータを取得する。
    struct GetParam {
      ParamId id;
    } PACKED get_param;

    // パラメータを設定する。
    // 設定するパラメータがPIDのゲインの場合は、ResetPidが送られるまで適用しない。
    struct SetParam {
      ParamId id;
      ParamValue value;
    } PACKED set_param;

    // PIDの積分項をリセットし、PIDのゲインを適用する。
    struct ResetPid {
    } PACKED reset_pid;

    // 目標速度を設定する
    struct SetTargetVelocity {
      int16_t vx; // 前方向の速度 [mm/s]
      int16_t vy; // 左方向の速度 [mm/s]
      int16_t ang_vel; // 上から見て半時計回り方向の角速度 [mrad/s] (ミリラジアン毎秒)
    } PACKED set_target_velocity;

    // 段差の乗り越えを開始する
    struct GetOver {
      // 乗り越える段差の種類
      enum StepKind : uint8_t {
        SMALL, // 地区大会
        LARGE, // 全国大会
        CENTER, // 中央の段差
      } step_kind;
    } PACKED get_over;

    // ドンファン
    struct SetDonfanCmd {
      int16_t cmd; // +展開 -曲がる
    } PACKED set_donfan_cmd;

    // ロジャー
    struct SetExpanderCmd {
      int16_t cmd; // +展開
    } PACKED set_expander_cmd;

    // ペチペチくん
    struct SetCollectorCmd {
      int16_t cmd; // +回収
    } PACKED set_collector_cmd;

    // お助けロジャー
    struct SetArmCmd {
      uint8_t index; // 操作する機構の番号
      int16_t expander_cmd; // +展開 -縮小
      int16_t tilt_cmd; // +正転 -逆転
    } PACKED set_arm_cmd;
  };
} PACKED;


// マイコンからPCへ送信するメッセージ
struct Feedback {
  static constexpr int ID = 201;

  enum class Tag : uint8_t {
    PARAM_EVENT,
    GET_PARAM_RESPONSE,
    ODOMETRY,
    GET_OVER_DONE,
    HEARTBEAT,
  };

  Tag tag;

  union {
    // パラメータのイベント
    // パラメータが設定されたときに送る
    struct ParamEvent {
      ParamId id;
      ParamValue value;
    } PACKED param_event;

    // `get_param`に対する返信
    struct GetParamResponse {
      ParamId id;
      ParamValue value;
    } PACKED get_param_response;

    // オドメトリの情報
    struct Odometry {
      int16_t vx; // 前方向の速度 [mm/s]
      int16_t vy; // 左方向の速度 [mm/s]
      int16_t ang_vel; // 上から見て半時計回り方向の角速度 [mrad/s] (ミリラジアン毎秒)
    } PACKED odometry;
    
    // 段差の乗り越え終了時に送る
    struct GetOverDone {
    } PACKED get_over_done;

    // 通信が生きていることを確認するために、定期的に送信するメッセージ
    // １秒間隔で送る
    // このメッセージを２秒以上受信できなかったとき、通信が死んだと判断する
    struct Heartbeat {
    } PACKED heartbeat;
  };
} PACKED;

static_assert(sizeof(Command) <= 8);
static_assert(sizeof(Feedback) <= 8);

#endif
