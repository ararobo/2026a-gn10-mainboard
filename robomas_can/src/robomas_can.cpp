#include "robomas_can/robomas_can.hpp"

#include <cstring>

namespace robomas_can {
RobomasCAN::RobomasCAN(gn10_can::CANBus& bus, float conversion)
    : bus_(bus), current_conversion_(conversion)
{
}

void RobomasCAN::set_current(uint8_t motor_number, float current_value)
{
    if (motor_number < 1 || motor_number > 8) return;

    current_value *= current_conversion_;
    motor_number -= 1;
    uint8_t group                        = motor_number / 4;
    uint8_t index                        = motor_number % 4;
    motor_current_[group].current[index] = current_value;
}

float RobomasCAN::get_current(uint8_t motor_number) const
{
    if (motor_number < 1 || motor_number > 8) return 0.0f;

    motor_number -= 1;
    uint8_t group = motor_number / 4;
    uint8_t index = motor_number % 4;
    return motor_current_[group].current[index];
}

void RobomasCAN::send_data(uint16_t can_id)
{
    uint8_t can_number = (can_id & 0x01);
    frame_.id          = can_id;
    frame_.dlc         = 8;  // robomasは8byte固定

    // 送るデータはfloatじゃだめなので型変換を行う
    for (int i = 0; i < 4; i++) {
        int16_t val =
            __builtin_bswap16(static_cast<int16_t>(motor_current_[can_number].current[i]));
        memcpy(&frame_.data[i * 2], &val, 2);  // 2byteだからframe×2 頭で代入
    }

    bus_.send_frame(frame_);
}

}  // namespace robomas_can
