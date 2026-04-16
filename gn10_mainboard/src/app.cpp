#include "gn10_mainboard/app.hpp"

#include "drivers/stm32_fdcan/driver_stm32_fdcan.hpp"
#include "fdcan.h"
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/motor_driver_client.hpp"
#include "gn10_can/devices/robot_control_hub_server.hpp"
#include "gn10_mainboard/fdcan_driver.hpp"
#include "gn10_mainboard/four_wheel_omni.hpp"
#include "gn10_mainboard/pid.hpp"
#include "robomas_can/c620_can.hpp"
#include "wiznet_ether/robot_ethernet.hpp"
#include "wiznet_ether/serial_printf.hpp"

namespace {

constexpr uint32_t k_heartbeat_toggle_interval_ms = 500;

uint32_t heartbeat_last_toggle_time_ms = 0;

/**
 * @brief Toggle heartbeat LED at a fixed interval.
 */
void update_heartbeat_led()
{
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - heartbeat_last_toggle_time_ms) >= k_heartbeat_toggle_interval_ms) {
        heartbeat_last_toggle_time_ms = now_ms;
        HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    }
}

}  // namespace

gn10_can::drivers::DriverSTM32FDCAN can1_driver(&hfdcan1);
gn10_can::drivers::FDCANDriver fdcan2_driver(&hfdcan2);

gn10_can::FDCANBus fdcan2_bus(fdcan2_driver);

robomas_can::C620CAN wheel_esc(can1_driver);

gn10_can::devices::RobotControlHubServer<operation_data_t, feedback_data_t> robot_control_hub(
    fdcan2_bus, 0
);

FourWheelOmni omni(0.2f, 0.05f);

gn10_motor::PIDConfig<float> pid_config_wheel_fr;
gn10_motor::PIDConfig<float> pid_config_wheel_fl;
gn10_motor::PIDConfig<float> pid_config_wheel_bl;
gn10_motor::PIDConfig<float> pid_config_wheel_br;

gn10_motor::PID<float> pid_wheel_fr(pid_config_wheel_fr);
gn10_motor::PID<float> pid_wheel_fl(pid_config_wheel_fl);
gn10_motor::PID<float> pid_wheel_bl(pid_config_wheel_bl);
gn10_motor::PID<float> pid_wheel_br(pid_config_wheel_br);

operation_data_t operation;
float wheel_angular_velocity_fr = 0.0f;
float wheel_angular_velocity_fl = 0.0f;
float wheel_angular_velocity_bl = 0.0f;
float wheel_angular_velocity_br = 0.0f;

float wheel_angular_velocity_fr_feedback = 0.0f;
float wheel_angular_velocity_fl_feedback = 0.0f;
float wheel_angular_velocity_bl_feedback = 0.0f;
float wheel_angular_velocity_br_feedback = 0.0f;

/**
 * @brief Initialize CAN and mainboard application state.
 */
void setup()
{
    can1_driver.init();
    fdcan2_driver.init();

    pid_config_wheel_fr.kp           = 1.709f;
    pid_config_wheel_fr.ki           = 0.0f;
    pid_config_wheel_fr.kd           = 0.0f;
    pid_config_wheel_fr.output_limit = 20.0f;
    pid_wheel_fr.update_config(pid_config_wheel_fr);
    pid_config_wheel_fl.kp           = 1.709f;
    pid_config_wheel_fl.ki           = 0.0f;
    pid_config_wheel_fl.kd           = 0.0f;
    pid_config_wheel_fl.output_limit = 20.0f;
    pid_wheel_fl.update_config(pid_config_wheel_fl);
    pid_config_wheel_bl.kp           = 1.709f;
    pid_config_wheel_bl.ki           = 0.0f;
    pid_config_wheel_bl.kd           = 0.0f;
    pid_config_wheel_bl.output_limit = 20.0f;
    pid_wheel_bl.update_config(pid_config_wheel_bl);
    pid_config_wheel_br.kp           = 1.709f;
    pid_config_wheel_br.ki           = 0.0f;
    pid_config_wheel_br.kd           = 0.0f;
    pid_config_wheel_br.output_limit = 20.0f;
    pid_wheel_br.update_config(pid_config_wheel_br);

    heartbeat_last_toggle_time_ms = HAL_GetTick();
}

/**
 * @brief Run one control cycle and update status heartbeat LED.
 */
void loop()
{
    if (robot_control_hub.get_command(operation)) {
    }
    omni.convert(operation.vx, operation.vy, operation.omega, 0.0f);
    omni.getWheelAngularVelocity(
        &wheel_angular_velocity_fr,
        &wheel_angular_velocity_fl,
        &wheel_angular_velocity_bl,
        &wheel_angular_velocity_br
    );

    // serial_printf("%f\n", operation.vy);
    // serial_printf("%f\n", wheel_angular_velocity_fr);

    wheel_angular_velocity_fr_feedback =
        2.0f * 3.1415f * (float)wheel_esc.get_feedback_speed(0) / 60.0f;
    wheel_angular_velocity_fl_feedback =
        2.0f * 3.1415f * (float)wheel_esc.get_feedback_speed(1) / 60.0f;
    wheel_angular_velocity_bl_feedback =
        2.0f * 3.1415f * (float)wheel_esc.get_feedback_speed(2) / 60.0f;
    wheel_angular_velocity_br_feedback =
        2.0f * 3.1415f * (float)wheel_esc.get_feedback_speed(3) / 60.0f;

    float wheel_currents[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    wheel_currents[0] =
        pid_wheel_fr.update(wheel_angular_velocity_fr, wheel_angular_velocity_fr_feedback, 0.01f) *
        0.1;
    wheel_currents[1] =
        pid_wheel_fl.update(wheel_angular_velocity_fl, wheel_angular_velocity_fl_feedback, 0.01f);
    wheel_currents[2] =
        pid_wheel_bl.update(wheel_angular_velocity_bl, wheel_angular_velocity_bl_feedback, 0.01f);
    wheel_currents[3] =
        pid_wheel_br.update(wheel_angular_velocity_br, wheel_angular_velocity_br_feedback, 0.01f);

    wheel_esc.set_current_can1(
        wheel_currents[0], wheel_currents[1], wheel_currents[2], wheel_currents[3]
    );

    serial_printf("%f,fe\n", wheel_currents[0]);
    // serial_printf("%d\n", wheel_angular_velocity_br);
    //  serial_printf("%d", wheel_angular_velocity_bl);
    //  serial_printf("%d\n", wheel_angular_velocity_fr);
    //  serial_printf("%d", wheel_angular_velocity_fl);
    // serial_printf("%f\n", wheel_currents[0]);
    // serial_printf("%f\n", wheel_angular_velocity_fr);  // 上と同じ値
    // serial_printf("%f\n", wheel_angular_velocity_fr_feedback);  // feedbackは0
    // serial_printf("%f\n", operation.vx); //最大値1
    //  serial_printf("%f\n", operation.vy);  // 最大値1
    // serial_printf("%f\n", operation.omega);
    update_heartbeat_led();
    //     robomas用の
    HAL_Delay(10);
}
extern "C" {
// C言語側の関数のオーバーライド
/**
 * @brief Receive callback for FDCAN FIFO0.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == hfdcan1.Instance) {
        gn10_can::CANFrame rx_frame;
        can1_driver.receive(rx_frame);
        wheel_esc.receive_data(rx_frame.id, rx_frame.data.data());
    } else if (hfdcan->Instance == hfdcan2.Instance) {
        fdcan2_bus.update();
    }
}
}