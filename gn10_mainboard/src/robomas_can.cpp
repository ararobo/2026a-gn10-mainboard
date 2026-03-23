#include "gn10_mainboard/robomas_can.hpp"

#include <cstring>

namespace robomaster {

// コンストラクタの引数にbusを設定することによって、他のバスとかぶらないようにする。
robomas_can::robomas_can(gn10_can::CANBus& bus) : bus_(bus) {}

void robomas_can::send_data(uint16_t can_id, uint8_t* data)
{
    frame.id  = can_id;
    frame.dlc = 8;  // robomasは8byte固定
    for (int i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    bus_.send_frame(frame);
}

void robomas_can::send()
{
    send_data(SEND_CANID_1_4, reinterpret_cast<uint8_t*>(&current_1));

    send_data(SEND_CANID_5_8, reinterpret_cast<uint8_t*>(&current_2));
}

void robomas_can::receive_data(uint16_t can_id, uint8_t data[8])
{
    switch (can_id) {
        case motor_1:
            memcpy(&feedback[0], data, 8);
            process_data(&feedback[0]);
            break;
        case motor_2:
            memcpy(&feedback[1], data, 8);
            process_data(&feedback[1]);
            break;
        case motor_3:
            memcpy(&feedback[2], data, 8);
            process_data(&feedback[2]);
            break;
        case motor_4:
            memcpy(&feedback[3], data, 8);
            process_data(&feedback[3]);
            break;
        case motor_5:
            memcpy(&feedback[4], data, 8);
            process_data(&feedback[4]);
            break;
        case motor_6:
            memcpy(&feedback[5], data, 8);
            process_data(&feedback[5]);
            break;
        case motor_7:
            memcpy(&feedback[6], data, 8);
            process_data(&feedback[6]);
            break;
        case motor_8:
            memcpy(&feedback[7], data, 8);
            process_data(&feedback[7]);
            break;
        default:
            break;
    }
}

void robomas_can::process_data(MotorFeedback* feedback)
{
    feedback->angle   = __builtin_bswap16(feedback->angle);
    feedback->speed   = __builtin_bswap16(feedback->speed);
    feedback->current = __builtin_bswap16(feedback->current);
}

void robomas_can::set_current_group1(
    int16_t motor1_current, int16_t motor2_current, int16_t motor3_current, int16_t motor4_current
)
{
    current_1.motor1_current = __builtin_bswap16(motor1_current);
    current_1.motor2_current = __builtin_bswap16(motor2_current);
    current_1.motor3_current = __builtin_bswap16(motor3_current);
    current_1.motor4_current = __builtin_bswap16(motor4_current);
}

void robomas_can::set_current_group2(
    int16_t motor5_current, int16_t motor6_current, int16_t motor7_current, int16_t motor8_current
)
{
    current_2.motor5_current = __builtin_bswap16(motor5_current);
    current_2.motor6_current = __builtin_bswap16(motor6_current);
    current_2.motor7_current = __builtin_bswap16(motor7_current);
    current_2.motor8_current = __builtin_bswap16(motor8_current);
}
}  // namespace robomaster