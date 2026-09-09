/**
 * @file robot_data_config.hpp
 * @author tmcit-ararobo-2026a
 * @brief ロボットの通信データ構造体定義
 * @version 2.2
 * @date 2026-09-07
 *
 * @copyright Copyright (c) 2026
 *
 * socket_cmd (port:26574)
 *  |-  operation    pc          ->  main-board
 *  |-  feedback     main-board  ->  pc
 *
 * socket_teleop (port:10410)
 *  |-  teleop            robo-con    ->  pc ( main (in debug_mode))
 *
 * socket_debug (port:39244)
 *  |-  pc_debug          main-board  ->  pc
 *  |-  main_debug        pc          ->  main-board
 */
#pragma once
#include <stdint.h>

namespace robot_config {

namespace header {
constexpr uint8_t operation  = 0xAB;
constexpr uint8_t feedback   = 0x55;
constexpr uint8_t teleop     = 0xAA;
constexpr uint8_t pc_debug   = 0x38;
constexpr uint8_t main_debug = 0x2A;
}  // namespace header

namespace port {
constexpr uint16_t cmd    = 26574;
constexpr uint16_t teleop = 10410;
constexpr uint16_t debug  = 39244;
}  // namespace port

namespace ip {
constexpr uint8_t mainboard[] = {192, 168, 3, 2};
constexpr uint8_t pc_robot[]  = {192, 168, 3, 1};
constexpr uint8_t pc_wifi[]   = {192, 168, 2, 1};
constexpr uint8_t teleop[]    = {192, 168, 2, 2};
}  // namespace ip

/**
 * @brief ロボットの動作司令値 32byte
 *
 */
struct command_t {
    // 識別ヘッダー 1byte
    uint8_t header;
    uint8_t reserved[3];
    float move_bucket_angle_yaw_rad;
    float bucket1_angle_yaw_rad;
    float bucket2_angle_yaw_rad;
    float bucket3_angle_yaw_rad;
    float flag_angle_yaw_rad;
    float desk_angle_yaw_rad;
} __attribute__((__packed__));

union command_u {
    command_t value;                    // 操作データ
    uint8_t binary[sizeof(command_t)];  // 送信バイト配列
} __attribute__((__packed__));

static_assert(sizeof(command_t) == 28);

/**
 * @brief ロボットのセンサ値などのフィードバック
 *
 */
struct feedback_t {
    uint8_t header;    // ヘッダー
    uint8_t sequence;  // シーケンス番号
    // 電源周り
    bool emergency_stop_enabled;
    bool over_current;
    float drive_battery_voltages;
    float logic_battery_voltages[2];
    float drive_current;
    // 各アクチュエータ
    float wheel_angular_velocity[3];  // 0:front 1:left 2:right
    float belt_launcher_velocity;     // [m/s]
    float loading_belt_angle;         // 装填機構のプーリー角度[rad]
    float bucket_arm_height;          // バケツアームの高さ[m]
} __attribute__((__packed__));

union feedback_u {
    feedback_t value;
    uint8_t binary[sizeof(feedback_t)];
} __attribute__((__packed__));

static_assert(sizeof(feedback_t) == 44);

/**
 * @brief 操縦デバイスのレバーの傾きと押し込み
 *
 */
enum class LeverPosition : uint8_t {
    FRONT,
    RIGHT,
    RIGHT_DEEP,
    LEFT,
    LEFT_DEEP,
    PUSH,
};

/**
 * @brief ロボットの操縦信号値
 *
 */
struct teleop_t {
    uint8_t header;  // 認識番号

    struct {
        int8_t stick_right[2];             // 0:x, 1:y
        int8_t stick_left[2];              // 0:x, 1:y
    } __attribute__((__packed__)) analog;  // 4byte

    struct {
        LeverPosition lever_right : 3;
        LeverPosition lever_left  : 3;
        uint8_t stick_push_right  : 1;
        uint8_t stick_push_left   : 1;
        uint8_t left_up           : 1;
        uint8_t left_down         : 1;
        uint8_t left_right        : 1;
        uint8_t left_left         : 1;
        uint8_t right_right       : 1;
        uint8_t right_up          : 1;
        uint8_t right_down        : 1;
        uint8_t reserved          : 1;
    } __attribute__((__packed__)) buttons;  // 2byte

    /**
     * checksum以外を除いた7Byteの和の補数
     * ただし計算結果の8bitより大きい値は切り捨て
     */
    uint8_t data_checksum;
} __attribute__((__packed__));

union teleop_u {
    teleop_t value;
    uint8_t binary[sizeof(teleop_t)];
} __attribute__((__packed__));

static_assert(sizeof(teleop_t) == 8);

/**
 * @brief PCのデバッグ用通信（再起動やスクリプト開始など）
 *
 */
struct debug_pc_t {
    uint8_t header;  // ヘッダー
    bool jetson_restart;
    bool jetson_shutdown;
    bool node_start;
    bool node_stop;
} __attribute__((__packed__));

union debug_pc_u {
    debug_pc_t value;
    uint8_t binary[sizeof(debug_pc_t)];
} __attribute__((__packed__));

/**
 * @brief メイン基板(RobotControlHub)のデバッグ用通信（制御モード切り替えなど）
 *
 */
struct debug_main_t {
    uint8_t header;
} __attribute__((__packed__));

union debug_main_u {
    debug_main_t value;
    uint8_t binary[sizeof(debug_main_t)];
} __attribute__((__packed__));

}  // namespace robot_config