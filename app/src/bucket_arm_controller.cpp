#include "app/bucket_arm_controller.hpp"

float BucketArmController::height_motor_output(bool up, bool down) const
{
    // 最大回転角度
    const float max_angle = (height_max_ - height_min_) / pulley_radius_;
    // 同時入力時は停止
    if (up == down) {
        return 0.0f;
    }
    // 上昇時はリミットスイッチが作動するため上限は設けない
    if (up) {
        return -height_adjustment_velocity_ratio_;
    }
    // 下限に達した際、その方向への移動のみを停止
    if (down && height_motor_angle_ < max_angle) {
        return height_adjustment_velocity_ratio_;
    }
    return 0.0f;
}

float BucketArmController::hold_motor_output(bool hold) const
{
    if (hold) {
        return -hold_current_;
    } else {
        return release_current_;
    }
}