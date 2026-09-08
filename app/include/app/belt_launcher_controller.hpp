#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

/**
 * @brief ベルト直動式射出機構の制御クラス
 *
 */
class BeltLauncherController
{
public:
    /**
     * @brief コンストラクタ
     *
     * @param max_velocity 最大射出速度[m/s]
     * @param min_velocity 最小射出速度[m/s]
     * あまりに速度が小さく射出できないことがないように設定する
     */
    BeltLauncherController(float max_velocity, float min_velocity)
        : max_velocity_(max_velocity), min_velocity_(min_velocity)
    {
        state = State::Uninitialized;
    }

    /**
     * @brief 無操作時の初期速度を設定
     *
     * @param velocity 目標射出速度[m/s]
     */
    void set_default_velocity(float velocity)
    {
        target_velocity_ = std::clamp(velocity, min_velocity_, max_velocity_);
    }

    /**
     * @brief 射出速度調整の度合い
     *
     * @param adjustment_amount 一回の調整でどれぐらい変化させるか[m/s]
     */
    void set_velocity_adjustment_amount(float adjustment_amount)
    {
        velocity_adjustment_amount_ = adjustment_amount;
    }

    /**
     * @brief 再装填時の角度変化量
     *
     * @param delta_angle １装填分の角度変化[rad]
     */
    void set_reload_angle_delta(float delta_angle)
    {
        reload_angle_delta_ = delta_angle;
    }

    /**
     * @brief 原点復帰から再装填までの遅延
     *
     * @param time_ms 遅延時間[ms]
     */
    void set_reload_delay_ms(uint32_t time_ms)
    {
        reload_delay_ms_ = time_ms;
    }

    /**
     * @brief 原点復帰したことを記録
     *
     * @param now_ms 原点復帰した時間[ms]
     */
    void set_initial_point(uint32_t now_ms);

    /**
     * @brief 速度調整
     *
     * @param up 速度上昇
     * @param down 速度低下
     */
    void update_velocity(bool up, bool down)
    {
        if (up) {
            target_velocity_ += velocity_adjustment_amount_;
        }
        if (down) {
            target_velocity_ -= velocity_adjustment_amount_;
        }
        target_velocity_ = std::clamp(target_velocity_, min_velocity_, max_velocity_);
    }

    /**
     * @brief 射出するか
     *
     * @param target_velocity 目標射出速度[m/s]
     * @return true 射出して
     * @return false 射出しないで
     */
    bool fire(float& target_velocity);

    /**
     * @brief 装填するか
     *
     * @param target_angle 装填機構に与える目標角度
     * @param now_ms 現在の時間[ms]
     * @return true 装填して
     * @return false 装填しないで
     *
     * @note
     * 装填しない場合でもtarget_angleが変化すると装填されるので注意。内部では装填時のみ変化するようにする。
     */
    bool load_a_cloth(float& target_angle, uint32_t now_ms);

private:
    float max_velocity_{};
    float min_velocity_{};
    float velocity_adjustment_amount_{};
    float target_velocity_{};
    float cloth_loader_target_angle_{};
    float reload_angle_delta_ = (float)M_PI * 2.0f / 3.0f;
    uint32_t reload_delay_ms_{};
    uint32_t last_initial_point_time_ms_{};
    bool wait_for_reload_{};
    enum class State {
        Uninitialized,
        InitialPoint,
        ClothLoaded,
        Firing,
    } state;
};