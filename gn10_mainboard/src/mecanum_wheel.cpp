#include "gn10_mainboard/mecanum_wheel.hpp"

MecanumWheel::MecanumWheel(
    float chassis_width, float chassis_length, float wheel_radius, float mecanum_angle
)
    : chassis_width(chassis_width),
      chassis_length(chassis_length),
      wheel_radius(wheel_radius),
      mecanum_angle(mecanum_angle)
{
    chassis_diagonal = sqrt(pow(chassis_width, 2) + pow(chassis_length, 2));
    mecanum_cos      = cos(mecanum_angle * M_PI / 180.0f);
    mecanum_sin      = sin(mecanum_angle * M_PI / 180.0f);
}

void MecanumWheel::convert_coord(float* vx, float* vy, float theta)
{
    *vx = cos(theta) * *vx + sin(theta) * *vy;   // x軸方向の速度
    *vy = -sin(theta) * *vx + cos(theta) * *vy;  // y軸方向の速度
}

void MecanumWheel::calculate_wheel_speed(float vx, float vy, float omega)
{
    wheel_velocity[0] =
        (-vx / mecanum_cos + vy / mecanum_sin + omega * (chassis_diagonal / 2.0f));  // 右前
    wheel_velocity[1] =
        (-vx / mecanum_cos - vy / mecanum_sin + omega * (chassis_diagonal / 2.0f));  // 左前
    wheel_velocity[2] =
        (vx / mecanum_cos - vy / mecanum_sin + omega * (chassis_diagonal / 2.0f));  // 左後
    wheel_velocity[3] =
        (vx / mecanum_cos + vy / mecanum_sin + omega * (chassis_diagonal / 2.0f));  // 右後
}

void MecanumWheel::get_wheel_angular_velocity(
    float* front_right, float* front_left, float* back_left, float* back_right
)
{
    *front_right = wheel_velocity[0] / wheel_radius;  // 右前
    *front_left  = wheel_velocity[1] / wheel_radius;  // 左前
    *back_left   = wheel_velocity[2] / wheel_radius;  // 左後
    *back_right  = wheel_velocity[3] / wheel_radius;  // 右後
}

void MecanumWheel::get_wheel_velocity(
    float* front_right, float* front_left, float* back_left, float* back_right
)
{
    *front_right = wheel_velocity[0];  // 右前
    *front_left  = wheel_velocity[1];  // 左前
    *back_left   = wheel_velocity[2];  // 左後
    *back_right  = wheel_velocity[3];  // 右後
}