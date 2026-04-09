/**
 * @file ThreeWheelOmni.hpp
 * @author Tooda Haruki
 * @brief 機体の姿勢、速度ベクトル、角速度、から、それぞれのモータの速度、角速度を求めるクラス
 * @version 1.1
 * @date 2023-03-28
 */

#pragma once

#include <math.h>

class ThreeWheelOmni
{
private:
    // 機体（足回り）の定数、変数
    float chassis_radius_;           // 機体半径
    float chassis_angle_;            // 機体の傾き
    float chassis_angular_velocity_; // 機体の角速度
    float velocity_x_, velocity_y_;  // 機体の速度ベクトル
    float wheel_radius_;             // ホイールの半径

    // それぞれのホイールの回転速度
    float wheel_velocity_front_;
    float wheel_velocity_back_left_;
    float wheel_velocity_back_right_;

    // 度をラジアンにする関数
    float rad(float deg) { return deg * (M_PI / 180.0f); }

public:
    /**
     * @brief 機体半径と機体の傾きを初期化する
     *
     * @param[in] chassis_radius_ 機体の中心からホイールまでの距離[m]
     * @param[in] wheel_radius_ ホイールの半径[m]
     */
    ThreeWheelOmni(float chassis_radius_, float wheel_radius_)
        : chassis_radius_{chassis_radius_}, wheel_radius_{wheel_radius_} {}

    /**
     * @brief 速度ベクトル、角速度、機体の傾きから、それぞれのホイールの速度を求める
     *
     * @param[in] x 速度ベクトルのx成分[m/s]
     * @param[in] y 速度ベクトルのy成分[m/s]
     * @param[in] AngularVelocity 角速度[rad/s]
     * @param[in] Angle 機体の傾き[度]
     */
    void convert(float x, float y, float AngularVelocity, float Angle)
    {
        chassis_angular_velocity_ = AngularVelocity;
        //***引数を度にする場合ここを変更***
        chassis_angle_ = rad(Angle);
        // chassis_angle_ = Angle;

        // 回転行列の計算
        float R[2][2] = {{cos(chassis_angle_), sin(chassis_angle_)},
                         {-sin(chassis_angle_), cos(chassis_angle_)}};

        // 行列計算
        velocity_x_ = R[0][0] * x + R[0][1] * y;
        velocity_y_ = R[1][0] * x + R[1][1] * y;

        // 計算結果を代入
        wheel_velocity_front_ = -velocity_x_;
        wheel_velocity_back_left_ = cos(rad(60.0f)) * velocity_x_ - sin(rad(60.0f)) * velocity_y_;
        wheel_velocity_back_right_ = cos(rad(60.0f)) * velocity_x_ + sin(rad(60.0f)) * velocity_y_;

        // 角速度分の速度の加算
        wheel_velocity_front_ += chassis_radius_ * chassis_angular_velocity_;
        wheel_velocity_back_left_ += chassis_radius_ * chassis_angular_velocity_;
        wheel_velocity_back_right_ += chassis_radius_ * chassis_angular_velocity_;
    }

    /**
     * @brief それぞれのホイールの速度を取得する
     *
     * @param[out] wheel_velocity_front ホイールの速度front
     * @param[out] wheel_velocity_back_l ホイールの速度back_left
     * @param[out] wheel_velocity_back_r ホイールの速度back_right
     */
    void getWheelVelocity(float *wheel_velocity_front,
                          float *wheel_velocity_back_l,
                          float *wheel_velocity_back_r)
    {
        *wheel_velocity_front = wheel_velocity_front_;
        *wheel_velocity_back_l = wheel_velocity_back_left_;
        *wheel_velocity_back_r = wheel_velocity_back_right_;
    }

    /**
     * @brief それぞれのホイールの角速度を取得する
     *
     * @param[out] wheel_angular_velocity_front  ホイールの角速度front
     * @param[out] wheel_angular_velocity_back_l ホイールの角速度back_left
     * @param[out] wheel_angular_velocity_back_r ホイールの角速度back_right
     */
    void getWheelAngularVelocity(float *wheel_angular_velocity_front,
                                 float *wheel_angular_velocity_back_l,
                                 float *wheel_angular_velocity_back_r)
    {
        *wheel_angular_velocity_front = wheel_velocity_front_ / wheel_radius_;
        *wheel_angular_velocity_back_l = wheel_velocity_back_left_ / wheel_radius_;
        *wheel_angular_velocity_back_r = wheel_velocity_back_right_ / wheel_radius_;
    }
};