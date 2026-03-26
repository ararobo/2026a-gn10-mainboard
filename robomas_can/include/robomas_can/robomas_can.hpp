#pragma once
#include "gn10_can/core/can_bus.hpp"

namespace robomas_can {

// robomas共通の送信CANid
#define SEND_CANID 0x200

// M2006　M3508の制御フレーム（送信）
#define SEND_CANID_1_4 0x200
#define SEND_CANID_5_8 0x1FF

// motorに送る電流値を格納する配列
struct MotorCurrent {
    int16_t current[4];
} __attribute__((__packed__));

// c610とc620に対応するための基底クラス
class RobomasCAN
{
private:
    gn10_can::CANBus& bus_;
    gn10_can::CANFrame frame_;
    MotorCurrent current_[2];

public:
    /**
     * @brief CAN通信用クラスのコンストラクタ
     */
    RobomasCAN(gn10_can::CANBus& bus_);

    virtual ~RobomasCAN() = default;

    /**
     * @brief MotorCurrent用のget関数
     *
     * @param motor_number 何番目のロボマスモーターか判断する。
     * 値域は1-8。
     */
    virtual int16_t get_current(uint8_t motor_number) = 0;

    /**
     * @brief escからのfeedbackを受け取る関数
     * @param can_id feedback先のESCのid
     * @param data 受け取ったデータ
     */
    void receive_data(uint16_t can_id, uint8_t data[8]);

    /**
     * @brief escにデータを送る関数
     * @details get_currentで取得した電流値を送信します。
     */
    void send_data();
};
}  // namespace robomas_can
