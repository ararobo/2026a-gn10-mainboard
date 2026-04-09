#include "robomas_can/c620_can.hpp"

#include <cstring>

namespace robomas_can {

void C620CAN::receive_data(uint16_t can_id, uint8_t data[8])
{
    uint8_t motor_number = (can_id & 0x0F) - 1;
    if (motor_number >= 8) return;  // ガード

    memcpy(&feedback_[motor_number], data, 8);

    // c620はビックエンディアンなのでリトルエンディアンに変換
    feedback_[motor_number].angle     = __builtin_bswap16(feedback_[motor_number].angle);
    feedback_[motor_number].speed_rpm = __builtin_bswap16(feedback_[motor_number].speed_rpm);
    feedback_[motor_number].current   = __builtin_bswap16(feedback_[motor_number].current);
}
uint16_t C620CAN::get_feedback_angle(uint8_t motor_number) const
{
    if (motor_number > 7) return 0;
    return feedback_[motor_number].angle;
}

int16_t C620CAN::get_feedback_speed(uint8_t motor_number) const
{
    if (motor_number > 7) return 0;
    return feedback_[motor_number].speed_rpm;
}

int16_t C620CAN::get_feedback_current(uint8_t motor_number) const
{
    if (motor_number > 7) return 0;
    return feedback_[motor_number].current;
}

uint8_t C620CAN::get_feedback_temperature(uint8_t motor_number) const
{
    if (motor_number > 7) return 0;
    return feedback_[motor_number].temperature;
}
}  // namespace robomas_can