#include "app/conversion_command.hpp"

#include <algorithm>
#include <cstdint>

ConversionCommand::ConversionCommand() {}

/* belt */
void ConversionCommand::set_belt_vel_init(float belt_vel_init)
{
    belt_vel_      = belt_vel_init;
    belt_vel_init_ = belt_vel_init;
}

void ConversionCommand::set_belt_vel_adjust_value(float belt_vel_adjust_value)
{
    belt_vel_adjust_value_ = belt_vel_adjust_value;
}

/* bucket */
void ConversionCommand::set_bucket_hight_value(uint8_t bucket_hight_value)
{
    bucket_hight_value_ = bucket_hight_value;
}

void ConversionCommand::set_bucket_limit_value(int16_t bucket_limit_high, int16_t bucket_limit_low)
{
    bucket_limit_high_ = bucket_limit_high;
    bucket_limit_low_  = bucket_limit_low;
}

/* wheel */
void ConversionCommand::set_wheel_max_vel(float wheel_max_vel)
{
    wheel_max_vel_ = wheel_max_vel;
}

void ConversionCommand::set_angular_max_vel(float angular_max_vel)
{
    angular_max_vel_ = angular_max_vel;
}

/* wheel */
robot_config::command_t ConversionCommand::conversion(robot_config::teleop_t& teleop)
{
    // 足回り
    float x_vel;
    float y_vel;
    float angular_vel;

    // x velocity
    x_vel = std::clamp(static_cast<float>(teleop.analog.stick_left[0]) / INT8_MAX, -1.0f, 1.0f) *
            wheel_max_vel_;
    // y velocity
    y_vel = std::clamp(static_cast<float>(teleop.analog.stick_left[1]) / INT8_MAX, -1.0f, 1.0f) *
            wheel_max_vel_;

    // angular velocity
    angular_vel =
        std::clamp(static_cast<float>(teleop.analog.stick_right[0]) / INT8_MAX, -1.0f, 1.0f) *
        angular_max_vel_;

    command_.x_vel       = x_vel;
    command_.y_vel       = y_vel;
    command_.angular_vel = angular_vel;

    /*バケツ回収*/
    if (teleop.buttons.left_down) {
        if (teleop.buttons.right_up) {
            bucket_arm_hight_ += bucket_hight_value_;
        }
        if (teleop.buttons.right_down) {
            bucket_arm_hight_ -= bucket_hight_value_;
        }
        command_.bucket_arm_hold = teleop.buttons.right_right;
    }
    bucket_arm_hight_ = std::clamp(bucket_arm_hight_, bucket_limit_low_, bucket_limit_high_);
    command_.bucket_arm_hight = bucket_arm_hight_;

    /* ベルト直動*/
    // belt_throw
    if (!teleop.buttons.left_down) {
        command_.belt_throw = teleop.buttons.right_right && !teleop_last_.buttons.right_right;
        // belt出力調整
        if (teleop.buttons.right_up && !teleop_last_.buttons.right_up) {
            belt_vel_ += belt_vel_adjust_value_;
        }
        if (teleop.buttons.right_down && !teleop_last_.buttons.right_down) {
            belt_vel_ -= belt_vel_adjust_value_;
        }
    }

    belt_vel_         = std::clamp(belt_vel_, 0.0f, 1.0f);
    command_.belt_vel = belt_vel_;

    // belt_init処理
    if (!teleop.buttons.left_down) {
        command_.belt_init = teleop.buttons.stick_push_right;
    }

    // エアシリンダー
    command_.air_launcher_for_desk_r = false;
    command_.air_launcher_for_flag   = false;
    command_.air_launcher_for_desk_l = false;

    if (teleop.buttons.left_right) {
        command_.air_launcher_for_desk_r = true;
    }
    if (teleop.buttons.left_left) {
        command_.air_launcher_for_desk_l = true;
    }
    if (teleop.buttons.left_up) {
        command_.air_launcher_for_flag = true;
    }

    /*装填機構*/
    /*
    // もし自動で装填されなかったら手動で装填 feedbackなしなので
    if (teleop.buttons.stick_push_left) {
        command_.loading = true;
    } else {
        command_.loading = false;
    }*/

    teleop_last_ = teleop;
    return command_;
}
