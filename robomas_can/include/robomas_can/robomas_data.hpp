#pragma once
#include "gn10_can/core/can_frame.hpp"

namespace robomas_can {

// M2006　M3508の制御フレーム（送信）
constexpr int SEND_CANID_1_4 = 0x200;
constexpr int SEND_CANID_5_8 = 0x1FF;

// c610電流変換定数
constexpr float C610_CURRENT_CONVERSION = 100.0f;

// c620電流変換用定数
constexpr float C620_CURRENT_CONVERSION = 819.2f;

// motorに送る電流値を格納する配列
struct MotorCurrent {
    float current[4];
} __attribute__((__packed__));

// motorのtypeを格納する配列。
struct motor_type {
    uint8_t motor_type : 1;
} __attribute__((__packed__));

// c610とc620に対応するための基底クラス
class RobomasData
{
private:
    gn10_can::CANFrame frame_;
    MotorCurrent motor_current_[2];
    float current_conversion_[8];

    /**
     * @brief escにデータを送る関数
     * @details set_currentで設定した電流値を送信します。これは組み込まれている関数です
     */
    void send_currents(uint16_t can_id, uint8_t* data);

public:
    /**
     * @brief CAN通信用クラスのコンストラクタ
     *
     * @param current_conversion 電流変換定数
     */
    RobomasData(robomas_can::CANBus& bus);

    virtual ~RobomasData() = default;

    /**
     * @brief MotorCurrent用のget関数
     *
     * @param motor_number 何番目のロボマスモーターか判断する。
     * 値域は0-7。
     * @return 成功：設定された電流値
     * @return 失敗：０
     */
    float get_current(uint8_t motor_number) const;

    /**
     * @brief M2006かM3508か選ぶ
     *
     * @param motor_number 値域：0~7
     * @param motor_type falseがM3508　trueがM2006
     */
    void set_motor_type(uint8_t motor_number, bool motor_type);

    /**
     * @brief MotorCurrent用のsetter関数　電流値の値を代入します
     *
     * @param　motor_number 何話目のロボマスモーターか 値域：0~3
     * @param current 電流の値。
     */
    void set_current_can1(
        float motor0_current, float motor1_current, float motor2_current, float motor3_current
    );

    /**
     * @brief MotorCurrent用のsetter関数　電流値の値を代入します
     *
     * @param　motor_number 何話目のロボマスモーターか 値域：4~7
     * @param current 電流の値。
     */
    void set_current_can2(
        float motor4_current, float motor5_current, float motor6_current, float motor7_current
    );
    /**
     * @brief escからのfeedbackを受け取る関数
     * @param can_id feedback先のESCのid
     * @param data 受け取ったデータ
     */
    virtual void receive_data(uint16_t can_id, uint8_t data[8]) = 0;
};
}  // namespace robomas_can
