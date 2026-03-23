#pragma once
#include "gn10_can/core/can_bus.hpp"
// 定義
#define SEND_CANID 0x200

// M2006　M3508の制御フレーム（送信）
#define SEND_CANID_1_4 0x200
#define SEND_CANID_5_8 0x1FF

enum receive_id {
    motor_1 = 0x201,
    motor_2 = 0x202,
    motor_3 = 0x203,
    motor_4 = 0x204,
    motor_5 = 0x205,
    motor_6 = 0x206,
    motor_7 = 0x207,
    motor_8 = 0x208
};

struct MotorFeedback {
    uint16_t angle;
    int16_t speed;
    int16_t current;
    uint8_t temp;
    uint8_t spare;
} __attribute__((__packed__));

struct MotorCurrent_group1 {
    int16_t motor1_current = 0;
    int16_t motor2_current = 0;
    int16_t motor3_current = 0;
    int16_t motor4_current = 0;
} __attribute__((__packed__));

struct MotorCurrent_group2 {
    int16_t motor5_current = 0;
    int16_t motor6_current = 0;
    int16_t motor7_current = 0;
    int16_t motor8_current = 0;
} __attribute__((__packed__));

namespace robomaster {
class robomas_can
{
private:
    gn10_can::CANBus& bus_;
    gn10_can::CANFrame frame;
    MotorCurrent_group1 current_1;
    MotorCurrent_group2 current_2;

public:
    MotorFeedback feedback[8];
    robomas_can(gn10_can::CANBus& bus_);
    void send_data(uint16_t can_id, uint8_t* data);
    void receive_data(uint16_t can_id, uint8_t data[8]);
    void process_data(MotorFeedback* feedback);
    void send();
    void set_current_group1(
        int16_t motor1_current,
        int16_t motor2_current,
        int16_t motor3_current,
        int16_t motor4_current
    );

    void set_current_group2(
        int16_t motor5_current,
        int16_t motor6_current,
        int16_t motor7_current,
        int16_t motor8_current
    );
};
}  // namespace robomaster
