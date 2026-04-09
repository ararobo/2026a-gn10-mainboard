#include "robomas_can/robomas_can.hpp"

#include <cstring>

namespace robomas_can {
RobomasCAN::RobomasCAN(gn10_can::CANBus& bus) : bus_(bus)
{
    // 初期化処理
    for (int i = 0; i < 8; i++) {
        current_conversion_[i] = C620_CURRENT_CONVERSION;
    }
}

void RobomasCAN::set_motor_type(uint8_t motor_number, bool motor_type)
{
    if (motor_number > 7) return;

    if (motor_type == true) {
        current_conversion_[motor_number] = C610_CURRENT_CONVERSION;
    } else {
        current_conversion_[motor_number] = C620_CURRENT_CONVERSION;
    }
}

void RobomasCAN::set_current_can1(
    float motor0_current, float motor1_current, float motor2_current, float motor3_current
)
{
    motor_current_[0].current[0] = motor0_current * current_conversion_[0];
    motor_current_[0].current[1] = motor1_current * current_conversion_[1];
    motor_current_[0].current[2] = motor2_current * current_conversion_[2];
    motor_current_[0].current[3] = motor3_current * current_conversion_[3];
    send_currents(SEND_CANID_1_4, reinterpret_cast<uint8_t*>(&motor_current_[0]));
}

void RobomasCAN::set_current_can2(
    float motor4_current, float motor5_current, float motor6_current, float motor7_current
)
{
    motor_current_[1].current[0] = motor4_current * current_conversion_[4];
    motor_current_[1].current[1] = motor5_current * current_conversion_[5];
    motor_current_[1].current[2] = motor6_current * current_conversion_[6];
    motor_current_[1].current[3] = motor7_current * current_conversion_[7];
    send_currents(SEND_CANID_5_8, reinterpret_cast<uint8_t*>(&motor_current_[1]));
}

float RobomasCAN::get_current(uint8_t motor_number) const
{
    if (motor_number > 7) return 0.0f;

    uint8_t group = motor_number / 4;
    uint8_t index = motor_number % 4;
    return motor_current_[group].current[index];
}

void RobomasCAN::send_currents(uint16_t can_id, uint8_t* data)
{
    frame_.id  = can_id;
    frame_.dlc = 8;  // robomasは8byte固定
    for (int i = 0; i < 4; i++) {
        float current_float;
        memcpy(&current_float, &data[i * 4], 4);  // floatは4バイト

        int16_t current_int16 = __builtin_bswap16(static_cast<int16_t>(current_float));
        memcpy(&frame_.data[i * 2], &current_int16, 2);
    }

    bus_.send_frame(frame_);
}

}  // namespace robomas_can
