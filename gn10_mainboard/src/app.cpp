#include "gn10_mainboard/app.hpp"

#include "drivers/stm32_fdcan/driver_stm32_fdcan.hpp"
#include "fdcan.h"
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/motor_driver_client.hpp"
#include "gn10_mainboard/three_wheel_omni.hpp"
#include "wiznet_ether/robot_ethernet.hpp"
#include "robomas_can/c620_can.hpp"

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
gn10_can::CANBus can1_bus(can1_driver);
gn10_can::devices::MotorDriverClient wheel_front(can1_bus, 0);
gn10_can::devices::MotorDriverClient wheel_back_l(can1_bus, 1);
gn10_can::devices::MotorDriverClient wheel_back_r(can1_bus, 2);

gn10_can::devices::MotorConfig wheel_config;

RobotEthernet ethernet;
ThreeWheelOmni omni(0.2f, 0.06f);

operation_data_t operation;
float wheel_angular_velocity_front  = 0;
float wheel_angular_velocity_back_l = 0;
float wheel_angular_velocity_back_r = 0;

/**
 * @brief Initialize CAN and mainboard application state.
 */
void setup()
{
    can1_driver.init();
    ethernet.init();
    wheel_config.set_accel_ratio(1.0f);
    wheel_config.set_max_duty_ratio(1.0f);
    wheel_front.set_init(wheel_config);
    wheel_back_l.set_init(wheel_config);

    heartbeat_last_toggle_time_ms = HAL_GetTick();
}

/**
 * @brief Run one control cycle and update status heartbeat LED.
 */
void loop()
{
    ethernet.receive_operation_data(&operation);
    omni.convert(operation.vx, operation.vy, operation.omega, 0.0f);
    omni.getWheelAngularVelocity(
        &wheel_angular_velocity_front,
        &wheel_angular_velocity_back_l,
        &wheel_angular_velocity_back_r
    );
    wheel_front.set_target(wheel_angular_velocity_front);
    wheel_back_l.set_target(wheel_angular_velocity_back_l);
    wheel_back_r.set_target(wheel_angular_velocity_back_r);
    update_heartbeat_led();
    // robomas用の
    HAL_Delay(10);
}
extern "C" {
// C言語側の関数のオーバーライド
/**
 * @brief Receive callback for FDCAN FIFO0.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    can1_bus.update();
}
}