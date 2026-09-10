#include "app/app.hpp"
// std
#include <cmath>
// STM32 HAL
#include "fdcan.h"
// gn10-can
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/esc_hub_client.hpp"
#include "gn10_can/devices/launcher_client.hpp"
#include "gn10_can/devices/led_client.hpp"
#include "gn10_can/devices/motor_driver_client.hpp"
#include "gn10_can/devices/power_manager_client.hpp"
#include "gn10_can/devices/robot_control_hub_server.hpp"
#include "gn10_can/devices/solenoid_driver_client.hpp"
// gn10-mainboard
#include "app/belt_launcher_controller.hpp"
#include "app/bucket_arm_controller.hpp"
#include "app/led_information.hpp"
#include "app/robot_ethernet.hpp"
#include "app/serial_printf.hpp"
#include "app/three_wheel_omni.hpp"
// others
#include "gn10_stm32_fdcan_driver/can_callback_helper.hpp"
#include "gn10_stm32_fdcan_driver/can_driver.hpp"
#include "gn10_stm32_fdcan_driver/fdcan_driver.hpp"

namespace {
/* ----------------- 定数 ----------------------*/
// 足回り
constexpr float LINER_VELOCITY_MAX   = 4.0f;
constexpr float ANGULAR_VELOCITY_MAX = 4.5f;
constexpr float WHEEL_PID_GAINS[3]   = {0.05f, 0.0f, 0.0f};
// ベルト直動
constexpr float BELT_LAUNCHER_MAX_VELOCITY        = 8.0f;
constexpr float BELT_LAUNCHER_MIN_VELOCITY        = 2.0f;
constexpr float BELT_LAUNCHER_DEFAULT_VELOCITY    = 4.0f;
constexpr float BELT_LAUNCHER_ADJUSTMENT_VELOCITY = 0.5f;
// 装填機構
constexpr float RELOAD_ANGLE_ADJUST = 0.9690f;
constexpr float RELOAD_ANGLE_DELTA  = -(float)M_PI * 2.0f / 3.0f * RELOAD_ANGLE_ADJUST;
constexpr uint32_t RELOAD_DELAY_MS  = 2000;
constexpr float RELOAD_PID_GAINS[3] = {-1.5f, 0.0f, 0.0f};
// バケツ用アーム
constexpr float BUCKET_ARM_HEIGHT_PULLEY_RADIUS = 0.04f;   // [m]
constexpr float BUCKET_ARM_HEIGHT_MAX           = 0.6f;    // [m]
constexpr float BUCKET_ARM_HEIGHT_MIN           = 0.075f;  // [m]
constexpr float BUCKET_ARM_HOLD_FORCE           = 1.0f;    // [A]
constexpr float BUCKET_ARM_RELEASE_FORCE        = 1.0f;    // [A]
// 機械定数
constexpr float M3508_GEAR_RATIO = 19.0f;
// 処理定数
constexpr uint32_t HEARTBEAT_TOGGLE_INTERVAL_MS = 500;
constexpr uint32_t FEEDBACK_INTERVAL_MS         = 100;
constexpr uint32_t ETHER_INIT_DELAY_MS          = 1000;
constexpr uint32_t TELEOP_TIMEOUT_MS            = 100;
/* ---------------------- gn10-can ---------------------- */
// Device Configuration
gn10_can::devices::MotorConfig motor_config_wheel;
gn10_can::devices::MotorConfig motor_config_hand;
gn10_can::devices::MotorConfig motor_config_belt;
gn10_can::devices::MotorConfig motor_config_loading;
gn10_can::devices::MotorConfig motor_config_arm_height;
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
gn10_can::devices::MotorDriverClient dc_arm_height(can1_bus, 0);
gn10_can::devices::PowerManagerClient drive_power_manager(fdcan2_bus, 0);
gn10_can::devices::PowerManagerClient logic_power_manager(fdcan2_bus, 1);
gn10_can::devices::LauncherClient belt_launcher_client(fdcan3_bus, 0);
gn10_can::devices::LEDClient<LEDInformation> led_client(fdcan2_bus, 2);

/* ---------------------------- ethernet --------------------------*/
// Ethernet
RobotEthernet ether;

/* ---------------------------- 運動学 ------------------------- */
ThreeWheelOmni omni(0.4f, 0.13f / 2.0f);

/* ----------------------- robot control --------------------------*/
// 装填・アーム出力値
std::array<float, 4> arm_hold_and_loading_target{0.0f, 0.0f, 0.0f, 0.0f};

// ベルト直動
BeltLauncherController belt_launcher_controller(
    BELT_LAUNCHER_MAX_VELOCITY, BELT_LAUNCHER_MIN_VELOCITY
);

// バケツアーム
bool dc_arm_height_encoder_initialized = false;
BucketArmController bucket_arm(
    BUCKET_ARM_HEIGHT_PULLEY_RADIUS, BUCKET_ARM_HEIGHT_MAX, BUCKET_ARM_HEIGHT_MIN
);

/* --------------------- コントローラー（teleop）との通信 ---------------------*/
robot_config::teleop_t teleop{};
robot_config::teleop_t last_teleop{};
uint32_t last_teleop_received_ms = 0;
bool teleop_timeout              = false;

/* --------------------- PCとの通信 -----------------------------*/
robot_config::debug_pc_t prev_debug_pc{};
robot_config::feedback_t robot_feedback{};
robot_config::command_t robot_command{};

/* ----------------------- LED --------------------------*/
LEDInformation led_info;

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
        led_client.send_display_info(led_info);
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

/**
 * @brief ロボット司令より各アクチュエータに司令を送る
 */
void command_robot_drivers()
{
    // 足回り
    float x_vel =
        std::clamp(static_cast<float>(teleop.analog.stick_left[0]) / INT8_MAX, -1.0f, 1.0f) *
        LINER_VELOCITY_MAX;
    float y_vel =
        std::clamp(static_cast<float>(teleop.analog.stick_left[1]) / INT8_MAX, -1.0f, 1.0f) *
        LINER_VELOCITY_MAX;
    float angular_vel =
        std::clamp(static_cast<float>(teleop.analog.stick_right[0]) / INT8_MAX, -1.0f, 1.0f) *
        ANGULAR_VELOCITY_MAX;
    omni.convert(-x_vel, y_vel, angular_vel, 0.0f);
    float front, right, left;
    omni.getWheelAngularVelocity(&front, &left, &right);
    std::array<float, 4> wheel_targets{
        front * M3508_GEAR_RATIO, left * M3508_GEAR_RATIO, right * M3508_GEAR_RATIO, 0.0f
    };
    esc_wheel.set_targets(wheel_targets.data());

    // ベルト直動
    if (teleop.buttons.stick_push_right && !last_teleop.buttons.stick_push_right) {
        belt_launcher_client.set_init();
    }
    belt_launcher_controller.update_velocity(
        teleop.buttons.right_up && !last_teleop.buttons.right_up,
        teleop.buttons.right_down && !last_teleop.buttons.right_down
    );
    led_info.belt_velocity = belt_launcher_controller.get_target_velocity();
    float belt_launcher_target_vel{};
    // 左下ボタンが押されていない間はベルト直動操作モード
    if (!teleop.buttons.left_down) {
        if (teleop.buttons.right_right && !last_teleop.buttons.right_right) {  // 射出操作
            if (belt_launcher_controller.fire(belt_launcher_target_vel)) {     // 射出できるかどうか
                belt_launcher_client.send_fire_command(belt_launcher_target_vel);
            }
        }
    }

    // エア射出
    std::array<bool, 8> solenoid_targets{};
    solenoid_targets[0] = teleop.buttons.left_up;     // 旗
    solenoid_targets[1] = teleop.buttons.left_right;  // 机右
    solenoid_targets[2] = teleop.buttons.left_left;   // 机左
    if (solenoid_targets[0] || solenoid_targets[1] || solenoid_targets[2]) {
        led_info.air_injection = true;
    } else {
        led_info.air_injection = false;
    }
    solenoid.set_target(solenoid_targets);

    // バケツ用アーム
    float arm_height_target = 0.0f;
    if (teleop.buttons.left_down) {
        arm_height_target =
            bucket_arm.height_motor_output(teleop.buttons.right_up, teleop.buttons.right_down);
        arm_hold_and_loading_target[1] = bucket_arm.hold_motor_output(teleop.buttons.right_right);
    } else {
        arm_hold_and_loading_target[1] = 0.0f;
    }

    // 装填
    if (belt_launcher_controller.load_a_cloth(arm_hold_and_loading_target[2], HAL_GetTick())) {
    }

    // CAN通信
    esc_arm_hold_and_loading.set_targets(arm_hold_and_loading_target.data());
    dc_arm_height.set_target(arm_height_target);
}

void receive_and_process_feedbacks()
{
    std::array<float, 4> wheel_feedbacks{};
    if (esc_wheel.get_feedbacks(wheel_feedbacks.data())) {
        robot_feedback.wheel_angular_velocity[0] = wheel_feedbacks[0];  // front
        robot_feedback.wheel_angular_velocity[1] = wheel_feedbacks[1];  // left
        robot_feedback.wheel_angular_velocity[2] = wheel_feedbacks[2];  // right
    }
    float belt_launcher_feedback_vel{};
    if (belt_launcher_client.get_velocity_feedback(belt_launcher_feedback_vel)) {
        robot_feedback.belt_launcher_velocity = belt_launcher_feedback_vel;
    }
    float belt_launcher_initial_angle{};
    if (belt_launcher_client.get_initial_point(belt_launcher_initial_angle)) {
        belt_launcher_controller.set_initial_point(HAL_GetTick());
        led_info.belt_initialization = true;
    }
    float belt_release_point_velocity{};
    if (belt_launcher_client.get_release_point(belt_release_point_velocity)) {
        led_info.belt_initialization = false;
    }
    std::array<float, 4> loading_feedback = {};
    if (esc_arm_hold_and_loading.get_feedbacks(loading_feedback.data())) {
        robot_feedback.loading_belt_angle = loading_feedback[2];
    }
    float latest_arm_height_motor_angle = -dc_arm_height.feedback_value();  // 降下方向を+とする
    bucket_arm.set_height_motor_angle(latest_arm_height_motor_angle);
    // ゼロ点合わせが済んだらエンコーダーの値から高さを計算してフィードバックに代入
    if (dc_arm_height_encoder_initialized) {
        robot_feedback.bucket_arm_height =
            bucket_arm.angle_to_height(latest_arm_height_motor_angle);
    } else {  // ゼロ点取りが済んでいない場合、リミットスイッチで最高点を設定する
        uint8_t dc_arm_height_limit_sw = dc_arm_height.limit_switches();
        if ((dc_arm_height_limit_sw & 0b1)) {
            dc_arm_height.set_init(motor_config_arm_height);
            dc_arm_height_encoder_initialized = true;
        }
        robot_feedback.bucket_arm_height = 0.0f;  // ゼロ点があっていない間は0とする。
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
        robot_feedback.logic_battery_voltages[0] = voltages[0];
        robot_feedback.logic_battery_voltages[1] = voltages[1];
    }
    led_info.battery_voltage[0] = robot_feedback.logic_battery_voltages[0];
    led_info.battery_voltage[1] = robot_feedback.logic_battery_voltages[1];
    led_info.battery_voltage[2] = robot_feedback.drive_battery_voltages;
}

void stop_all_actuators()
{
    std::array<float, 4> esc_target_zero{};
    esc_wheel.set_targets(esc_target_zero.data());
    esc_arm_hold_and_loading.set_targets(esc_target_zero.data());
    dc_arm_height.set_target(0.0f);
    std::array<bool, 8> solenoid_target_zero{};
    solenoid.set_target(solenoid_target_zero);
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

    motor_config_arm_height.set_max_duty_ratio(0.75f);
    motor_config_arm_height.set_reverse_limit_switch(true, 0);
    motor_config_arm_height.set_motor_type(gn10_can::devices::MotorType::DC);
    motor_config_arm_height.set_encoder_type(gn10_can::devices::EncoderType::IncrementalTotal);
    motor_config_arm_height.set_feedback_cycle(10);

    // Other device configuration
    drive_power_manager_config.sensor_rate_ms            = 100;
    drive_power_manager_config.use_remote_emergency_stop = false;
    logic_power_manager_config.sensor_rate_ms            = 100;
    logic_power_manager_config.use_remote_emergency_stop = false;

    // Initialize devices on the network
    for (uint8_t i = 0; i < 4; i++) {
        esc_wheel.set_init(i, motor_config_wheel);
        esc_wheel.set_gains(i, WHEEL_PID_GAINS[0], WHEEL_PID_GAINS[1], WHEEL_PID_GAINS[2], 0.0f);
    }
    esc_arm_hold_and_loading.set_init(1, motor_config_hand);

    motor_config_loading.set_motor_type(gn10_can::devices::MotorType::C610);
    motor_config_loading.set_encoder_type(gn10_can::devices::EncoderType::IncrementalTotal);
    motor_config_loading.set_max_duty_ratio(10.0f);
    esc_arm_hold_and_loading.set_init(2, motor_config_loading);
    esc_arm_hold_and_loading.set_gains(
        2, RELOAD_PID_GAINS[0], RELOAD_PID_GAINS[1], RELOAD_PID_GAINS[2], 0.0f
    );

    dc_arm_height.set_init(motor_config_arm_height);
    solenoid.set_init();
    drive_power_manager.set_init(drive_power_manager_config);
    logic_power_manager.set_init(logic_power_manager_config);

    // Initialize Ethernet
    ether.init();

    bucket_arm.set_height_adjustment_velocity_ratio(1.0f);
    bucket_arm.set_hold_force_by_current(BUCKET_ARM_HOLD_FORCE);
    bucket_arm.set_release_force_by_current(BUCKET_ARM_RELEASE_FORCE);

    belt_launcher_controller.set_default_velocity(BELT_LAUNCHER_DEFAULT_VELOCITY);
    belt_launcher_controller.set_velocity_adjustment_amount(BELT_LAUNCHER_ADJUSTMENT_VELOCITY);
    belt_launcher_controller.set_reload_delay_ms(RELOAD_DELAY_MS);
    belt_launcher_controller.set_reload_angle_delta(RELOAD_ANGLE_DELTA);
    // System setup
    heartbeat_last_toggle_time_ms = HAL_GetTick();
}

/**
 * @brief Run one control cycle and update status heartbeat LED.
 */
void loop()
{
    const uint32_t now_ms = HAL_GetTick();
    // 指令値取得
    if (ether.receive_teleop(teleop)) {
        teleop_timeout          = false;
        last_teleop_received_ms = now_ms;
        command_robot_drivers();
    } else if ((now_ms - last_teleop_received_ms) > TELEOP_TIMEOUT_MS && !teleop_timeout) {
        teleop_timeout = true;
        stop_all_actuators();
    }
    if (ether.receive_operation_data(robot_command)) {
    }

    // フィードバック処理
    receive_and_process_feedbacks();
    periodic_feedback();
    read_button_and_send_debug_pc_packet();
    last_teleop = teleop;

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
