#include "robomas_can/c610_can.hpp"

#include <cstring>

namespace robomas_can {
void C610CAN::receive_data(uint16_t can_id, uint8_t data[8])
{
    uint8_t motor_number = (can_id & 0x0F) - 1;
    if (motor_number >= 8) return;  // ガード

    memcpy(&feedback_[motor_number], data, 8);

    // c610はビックエンディアンなのでリトルエンディアンに変換
    feedback_[motor_number].angle   = __builtin_bswap16(feedback_[motor_number].angle);
    feedback_[motor_number].speed   = __builtin_bswap16(feedback_[motor_number].speed);
    feedback_[motor_number].current = __builtin_bswap16(feedback_[motor_number].current);
}

uint16_t C610CAN::get_feedback_angle(uint8_t motor_number) const
{
    if (motor_number < 1 || motor_number > 8) return 0;
    return feedback_[motor_number - 1].angle;
}

int16_t C610CAN::get_feedback_speed(uint8_t motor_number) const
{
    if (motor_number < 1 || motor_number > 8) return 0;
    return feedback_[motor_number - 1].speed;
}

int16_t C610CAN::get_feedback_current(uint8_t motor_number) const
{
    if (motor_number < 1 || motor_number > 8) return 0;
    return feedback_[motor_number - 1].current;
}
}  // namespace robomas_can
