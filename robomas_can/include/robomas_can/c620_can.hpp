#include "robomas_can.hpp"
namespace robomas_can {
namespace c620 {

// feedback data
struct c620_feedback {
    uint16_t angle;       // 値域:0~8191
    int16_t speed_rpm;    // 値域：-32768 ~ 32767 単位：rpm
    int16_t current;      // 値域：-16384~16384
    uint8_t temperature;  // 値域：0~255 単位：℃
    uint8_t reserved;     // 空きデータ
} __attribute__((__packed__));

// c620電流変換用定数
constexpr float C620_CURRENT_CONVERSION = 819.2f;
}  // namespace c620

class C620CAN : public RobomasCAN
{
private:
    c620::c620_feedback feedback_[8];

public:
    // コンストラクタ
    C620CAN(gn10_can::CANBus& bus) : RobomasCAN(bus, c620::C620_CURRENT_CONVERSION) {}

    // 純粋仮想関数の実装
    void receive_data(uint16_t can_id, uint8_t data[8]) override;

    /**
     * @brief feedbackのangleを読み取るgetter関数。
     *
     * @param motor_number 読みたい角度のデータがあるモーター番号 値域:1~8
     *
     * @return 値域：0-8192 右の範囲内の角度
     */
    uint16_t get_feedback_angle(uint8_t motor_number) const;

    /**
     * @brief feedbackのspeedを読み取るgetter関数。
     *
     * @param motor_number 読みたい回転速度のデータがあるモーター番号 値域:1~8
     *
     * @return 値域:-32768~32768 単位:rpm の回転速度
     */
    int16_t get_feedback_speed(uint8_t motor_number) const;

    /**
     * @brief feedbackのcurrentを読み取るgetter関数。
     *
     * @param motor_number 読みたいトルク電流値のデータがあるモーター番号 値域:1~8
     *
     * @return 値域：-16384~16384
     */
    int16_t get_feedback_current(uint8_t motor_number) const;

    /**
     * @brief feedbackのtemperatureを読み取るgetter関数。
     *
     * @param motor_number 読みたい摂氏温度のデータがあるモーター番号 値域:1~8
     *
     * @return 値域：0~255 単位:℃　の摂氏温度
     */
    uint8_t get_feedback_temperature(uint8_t motor_number) const;
};

}  // namespace robomas_can
