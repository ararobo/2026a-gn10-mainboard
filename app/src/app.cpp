#include "app/app.hpp"
// std
#include <cmath>
// STM32 HAL
#include "fdcan.h"
// gn10-can
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/esc_hub_client.hpp"
#include "gn10_can/devices/launcher_client.hpp"
#include "gn10_can/devices/motor_driver_client.hpp"
#include "gn10_can/devices/power_manager_client.hpp"
#include "gn10_can/devices/robot_control_hub_server.hpp"
#include "gn10_can/devices/solenoid_driver_client.hpp"
// gn10-mainboard
#include "app/belt_launcher_controller.hpp"
#include "app/bucket_arm_controller.hpp"
#include "app/conversion_command.hpp"
#include "app/robot_ethernet.hpp"
#include "app/serial_printf.hpp"
#include "app/three_wheel_omni.hpp"
// others
#include "gn10_stm32_fdcan_driver/can_callback_helper.hpp"
#include "gn10_stm32_fdcan_driver/can_driver.hpp"
#include "gn10_stm32_fdcan_driver/fdcan_driver.hpp"

namespace {
/* ----------------- 定数 ----------------------*/
constexpr float BELT_LAUNCHER_MAX_VELOCITY        = 1.0f;
constexpr float BELT_LAUNCHER_MIN_VELOCITY        = 0.1f;
constexpr float BELT_LAUNCHER_DEFAULT_VELOCITY    = 0.3f;
constexpr float BELT_LAUNCHER_ADJUSTMENT_VELOCITY = 0.005f;

constexpr float BUCKET_ARM_HEIGHT_PULLEY_RADIUS = 0.122f;  // [m]
constexpr float BUCKET_ARM_HEIGHT_MAX           = 1.0f;    // [m]
constexpr float BUCKET_ARM_HEIGHT_MIN           = 0.1f;    // [m]

constexpr float M3508_GEAR_RATIO = 19.0f;

constexpr uint32_t HEARTBEAT_TOGGLE_INTERVAL_MS = 500;
constexpr uint32_t FEEDBACK_INTERVAL_MS         = 100;
constexpr uint32_t ETHER_INIT_DELAY_MS          = 1000;
/* ---------------------- gn10-can ---------------------- */
// Device Configuration
gn10_can::devices::MotorConfig motor_config_wheel;
gn10_can::devices::MotorConfig motor_config_hand;
gn10_can::devices::MotorConfig motor_config_belt;
gn10_can::devices::MotorConfig motor_config_loading;
gn10_can::devices::MotorConfig motor_config_arm_hight;
gn10_can::devices::power_manager::Config drive_power_manager_config;
gn10_can::devices::power_manager::Config logic_power_manager_config;
// CAN Drivers
gn10_can::drivers::CANDriver can1_driver(&hfdcan1);
gn10_can::drivers::FDCANDriver fdcan2_driver(&hfdcan2);
gn10_can::drivers::FDCANDriver fdcan3_driver(&hfdcan3);
// CAN Bus
gn10_can::CANBus can1_bus(can1_driver);
gn10_can::FDCANBus fdcan2_bus(fdcan2_driver);
gn10_can::FDCANBus fdcan3_bus(fdcan3_driver);
// CAN Devices
gn10_can::devices::SolenoidDriverClient solenoid(can1_bus, 0);
gn10_can::devices::RobotControlHubServer<robot_config::command_t, robot_config::feedback_t>
    robot_control_hub(fdcan2_bus, 0);
gn10_can::devices::ESCHubClient esc_wheel(fdcan3_bus, 1);
gn10_can::devices::ESCHubClient esc_arm_hold_and_loading(fdcan3_bus, 2);
gn10_can::devices::MotorDriverClient dc_arm_hight(can1_bus, 0);
gn10_can::devices::PowerManagerClient drive_power_manager(fdcan2_bus, 0);
gn10_can::devices::PowerManagerClient logic_power_manager(fdcan2_bus, 1);
gn10_can::devices::LauncherClient belt_launcher_vesc(fdcan3_bus, 0);

/* ---------------------------- ethernet --------------------------*/
// Ethernet
RobotEthernet ether;

/* --------------------- ロボット司令 ----------------------------- */
ConversionCommand conversion;
robot_config::command_t last_command_{};

/* ---------------------------- 運動学 ------------------------- */
ThreeWheelOmni omni(0.4f, 0.13f / 2.0f);

/* ----------------------- robot control --------------------------*/
// 装填・アーム出力値
std::array<float, 4> arm_hold_and_loading_target{0.0f, 0.0f, 0.0f, 0.0f};
// 装填
uint8_t reload_count = 0;     // 装填回数
bool reload_success  = true;  // 装填成功
bool reload_enabled  = false;
uint32_t release_time_tick;

// ベルト直動
BeltLauncherController belt_launcher_controller(
    BELT_LAUNCHER_MAX_VELOCITY, BELT_LAUNCHER_MIN_VELOCITY
);

// バケツアーム
bool dc_arm_hight_encoder_initialized = false;
BucketArmController bucket_arm(
    BUCKET_ARM_HEIGHT_PULLEY_RADIUS, BUCKET_ARM_HEIGHT_MAX, BUCKET_ARM_HEIGHT_MIN
);

/* --------------------- コントローラー（teleop）との通信 ---------------------*/
robot_config::teleop_t teleop{};

/* --------------------- PCとの通信 -----------------------------*/
robot_config::debug_pc_t prev_debug_pc{};
robot_config::feedback_t robot_feedback{};

/* ------------------ Lチカ ----------------------- */
uint32_t heartbeat_last_toggle_time_ms = 0;
/**
 * @brief 一定周期のLEDトグル
 */
void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= HEARTBEAT_TOGGLE_INTERVAL_MS) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    }
}

uint32_t feedback_last_send_time_ms = 0;
/**
 * @brief 一定周期でフィードバックをEthernetで送信する
 *
 */
void periodic_feedback()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - feedback_last_send_time_ms) >= FEEDBACK_INTERVAL_MS) {
        feedback_last_send_time_ms = now_ms;
        if (ether.send_feedback_data(robot_feedback)) {
            robot_feedback.sequence++;
        }
    }
}

/**
 * @brief エンコーダー端子に接続したスイッチを操作することで、PCに電源やプログラム起動の命令を送る
 *
 */
void read_button_and_send_debug_pc_packet()
{
    // ボタンが押されたら送る処理
    robot_config::debug_pc_t current_debug_pc = {};
    current_debug_pc.jetson_restart =
        (HAL_GPIO_ReadPin(operation_button1_GPIO_Port, operation_button1_Pin) == GPIO_PIN_SET);
    current_debug_pc.jetson_shutdown =
        (HAL_GPIO_ReadPin(operation_button2_GPIO_Port, operation_button2_Pin) == GPIO_PIN_SET);
    current_debug_pc.node_start =
        (HAL_GPIO_ReadPin(operation_button3_GPIO_Port, operation_button3_Pin) == GPIO_PIN_SET);
    current_debug_pc.node_stop =
        (HAL_GPIO_ReadPin(operation_button4_GPIO_Port, operation_button4_Pin) == GPIO_PIN_SET);
    if (current_debug_pc.jetson_restart != prev_debug_pc.jetson_restart ||
        current_debug_pc.jetson_shutdown != prev_debug_pc.jetson_shutdown ||
        current_debug_pc.node_start != prev_debug_pc.node_start ||
        current_debug_pc.node_stop != prev_debug_pc.node_stop) {
        ether.send_pc_debug_data(current_debug_pc);
        prev_debug_pc = current_debug_pc;
    }
}

void reload_cloth()
{
    if (reload_count == 0) {
        motor_config_loading.set_max_duty_ratio(10.0f);
        motor_config_loading.set_motor_type(gn10_can::devices::MotorType::C610);
        motor_config_loading.set_encoder_type(gn10_can::devices::EncoderType::IncrementalTotal);

        esc_arm_hold_and_loading.set_init(2, motor_config_loading);
        esc_arm_hold_and_loading.set_gains(2, -1.0f, 0.0f, 0.0f, 0.0f);
    }
    reload_count++;
    reload_success = false;
}

/**
 * @brief ロボット司令より各アクチュエータに司令を送る
 *
 * @param command
 */
void command_robot_drivers(const robot_config::command_t& command)
{
    // 足回り
    omni.convert(-command.x_vel, command.y_vel, command.angular_vel, 0.0f);
    float front, right, left;
    omni.getWheelAngularVelocity(&front, &left, &right);
    std::array<float, 4> wheel_targets{
        front * M3508_GEAR_RATIO, left * M3508_GEAR_RATIO, right * M3508_GEAR_RATIO, 0.0f
    };
    esc_wheel.set_targets(wheel_targets.data());

    // ベルト直動
    if (command.belt_init) {
        initialized_vesc = true;
        vesc_hub.set_init(0, motor_config_belt);
    }
    if (command.belt_throw && initialized_vesc && !last_command_.belt_throw) {
        vesc_throwing = true;
    }

    std::array<float, 4> vesc_target{0.0f, 0.0f, 0.0f, 0.0f};
    if (vesc_throwing) {
        vesc_target[0] = command.belt_vel;
    }
    vesc_hub.set_targets(vesc_target.data());

    // エア射出
    std::array<bool, 8> solenoid_targets{};
    solenoid_targets[0] = command.air_launcher_for_flag;
    solenoid_targets[1] = command.air_launcher_for_desk_r;
    solenoid_targets[2] = command.air_launcher_for_desk_l;
    solenoid.set_target(solenoid_targets);
    // バケツ用アーム
    float arm_hight_target = 0.0f;
    if (teleop.buttons.left_down) {
        arm_hight_target =
            bucket_arm.height_motor_output(teleop.buttons.right_up, teleop.buttons.right_down);
        arm_hold_and_loading_target[1] = bucket_arm.hold_motor_output(teleop.buttons.right_right);
    }
    // 装填
    if (!reload_success) {
        arm_hold_and_loading_target[2] =
            -static_cast<float>(reload_count) * 3.14f * 2 / 3 * SOLVE_LOADING_DEVIATION;
        reload_success = true;
    }
    // CAN通信
    esc_arm_hold_and_loading.set_targets(arm_hold_and_loading_target.data());
    dc_arm_hight.set_target(arm_hight_target);
    last_command_ = command;
}

}  // namespace

/**
 * @brief Initialize CAN and mainboard application state.
 */
void setup()
{
    HAL_Delay(ETHER_INIT_DELAY_MS);

    // CAN initialization
    can1_driver.init();
    fdcan2_driver.init();
    fdcan3_driver.init();

    // Motor configuration
    motor_config_wheel.set_motor_type(gn10_can::devices::MotorType::C620);
    motor_config_wheel.set_encoder_type(gn10_can::devices::EncoderType::InternalIncremental);
    motor_config_wheel.set_max_duty_ratio(20.0f);
    motor_config_wheel.set_accel_ratio(1.0f);

    motor_config_hand.set_motor_type(gn10_can::devices::MotorType::C610);
    motor_config_hand.set_encoder_type(gn10_can::devices::EncoderType::None);  // 電流制御

    motor_config_belt.set_motor_type(gn10_can::devices::MotorType::VESC);

    motor_config_arm_hight.set_max_duty_ratio(0.75f);
    motor_config_arm_hight.set_reverse_limit_switch(true, 0);
    motor_config_arm_hight.set_motor_type(gn10_can::devices::MotorType::DC);
    motor_config_arm_hight.set_encoder_type(gn10_can::devices::EncoderType::IncrementalTotal);
    motor_config_arm_hight.set_feedback_cycle(10);

    // Other device configuration
    drive_power_manager_config.sensor_rate_ms            = 100;
    drive_power_manager_config.use_remote_emergency_stop = false;
    logic_power_manager_config.sensor_rate_ms            = 100;
    logic_power_manager_config.use_remote_emergency_stop = false;

    // Initialize devices on the network
    for (uint8_t i = 0; i < 4; i++) {
        esc_wheel.set_init(i, motor_config_wheel);
        esc_wheel.set_gains(i, 0.05f, 0.0f, 0.0f, 0.0f);
    }
    esc_arm_hold_and_loading.set_init(1, motor_config_hand);
    esc_arm_hold_and_loading.set_gains(1, 0.001f, 0.0f, 0.0f, 0.0f);

    dc_arm_hight.set_init(motor_config_arm_hight);
    solenoid.set_init();
    drive_power_manager.set_init(drive_power_manager_config);
    logic_power_manager.set_init(logic_power_manager_config);

    // Initialize Ethernet
    ether.init();

    // controller command setup
    conversion.set_belt_vel_init(0.3f);
    conversion.set_belt_vel_adjust_value(0.005f);

    conversion.set_bucket_hight_value(100);
    conversion.set_bucket_limit_value(11000, 0);

    conversion.set_wheel_max_vel(4.5f);
    conversion.set_angular_max_vel(4.5f);

    bucket_arm.set_height_adjustment_velocity_ratio(1.0f);
    bucket_arm.set_hold_force_by_current(0.01f);
    bucket_arm.set_release_force_by_current(0.005f);

    // System setup
    heartbeat_last_toggle_time_ms = HAL_GetTick();
}
std::array<float, 4> loading_feedback = {};

/**
 * @brief Run one control cycle and update status heartbeat LED.
 */
void loop()
{
    const uint32_t now_ms = HAL_GetTick();
    // 指令値取得
    if (ether.receive_teleop(teleop)) {
        robot_config::command_t command;
        command = conversion.conversion(teleop);
        command_robot_drivers(command);
    }
    // フィードバック処理
    std::array<float, 4> wheel_feedbacks{};
    if (esc_wheel.get_feedbacks(wheel_feedbacks.data())) {
        robot_feedback.wheel_angular_velocity[0] = wheel_feedbacks[0];  // front
        robot_feedback.wheel_angular_velocity[1] = wheel_feedbacks[1];  // left
        robot_feedback.wheel_angular_velocity[2] = wheel_feedbacks[2];  // right
    }
    if (vesc_hub.get_feedbacks(vesc_feedbacks.data())) {
        reload_enabled    = true;
        vesc_throwing     = false;
        release_time_tick = now_ms;
    }
    if (esc_arm_hold_and_loading.get_feedbacks(loading_feedback.data())) {
    }
    float latest_arm_hight_motor_angle = dc_arm_hight.feedback_value();
    bucket_arm.set_height_motor_angle(latest_arm_hight_motor_angle);
    // ゼロ点合わせが済んだらエンコーダーの値から高さを計算してフィードバックに代入
    if (dc_arm_hight_encoder_initialized) {
        robot_feedback.bucket_arm_hight = bucket_arm.angle_to_height(latest_arm_hight_motor_angle);
    } else {  // ゼロ点取りが済んでいない場合、リミットスイッチで最高点を設定する
        uint8_t dc_arm_hight_limit_sw = dc_arm_hight.limit_switches();
        if ((dc_arm_hight_limit_sw & 0b1)) {
            dc_arm_hight.set_init(motor_config_arm_hight);
            dc_arm_hight_encoder_initialized = true;
        }
        robot_feedback.bucket_arm_hight = 0.0f;  // ゼロ点があっていない間は0とする。
    }

    if (reload_enabled && (now_ms - release_time_tick >= RELOAD_DELAY_MS)) {
        reload_cloth();
        reload_enabled = false;
    }

    gn10_can::devices::power_manager::Sensor drive_power_sensor{};
    if (drive_power_manager.get_new_sensor(drive_power_sensor)) {
        robot_feedback.drive_battery_voltages = drive_power_sensor.voltage;
        robot_feedback.drive_current          = drive_power_sensor.current;
    }
    gn10_can::devices::power_manager::Status drive_power_status{};
    if (drive_power_manager.get_new_status(drive_power_status)) {
        robot_feedback.emergency_stop_enabled = drive_power_status.emergency_stop_enabled;
        robot_feedback.over_current           = drive_power_status.over_current;
    }
    std::array<float, 4> voltages;
    if (logic_power_manager.get_new_voltages(voltages)) {
        robot_feedback.logic_battery_voltages[1] = voltages[1];
        robot_feedback.logic_battery_voltages[2] = voltages[2];
        robot_feedback.logic_battery_voltages[3] = voltages[3];
    }

    read_button_and_send_debug_pc_packet();
    periodic_feedback();

    // Basic System Process
    update_heartbeat_led();
}

// ---------------------------- C language's functions override ----------------------------------
extern "C" {
/**
 * @brief Receive callback for FDCAN FIFO0.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    if (process_fdcan_fifo(hfdcan, &hfdcan1, can1_bus, FDCAN_RX_FIFO0)) return;
    if (process_fdcan_fifo(hfdcan, &hfdcan2, fdcan2_bus, FDCAN_RX_FIFO0)) return;
    if (process_fdcan_fifo(hfdcan, &hfdcan3, fdcan3_bus, FDCAN_RX_FIFO0)) return;
}

/**
 * @brief Receive callback for FDCAN FIFO1.
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo1ITs)
{
    (void)RxFifo1ITs;
    if (process_fdcan_fifo(hfdcan, &hfdcan1, can1_bus, FDCAN_RX_FIFO1)) return;
    if (process_fdcan_fifo(hfdcan, &hfdcan2, fdcan2_bus, FDCAN_RX_FIFO1)) return;
    if (process_fdcan_fifo(hfdcan, &hfdcan3, fdcan3_bus, FDCAN_RX_FIFO1)) return;
}
}
