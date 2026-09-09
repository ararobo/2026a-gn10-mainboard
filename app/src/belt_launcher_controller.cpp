#include "app/belt_launcher_controller.hpp"

void BeltLauncherController::set_initial_point(uint32_t now_ms)
{
    last_initial_point_time_ms_ = now_ms;
    // 発射中だった場合は再装填待ちに設定し、初期位置に戻ったことを記録
    if (state == State::Firing) {
        wait_for_reload_ = true;
        state            = State::InitialPoint;
    }
    // 初回ゼロ点合わせでは再装填しない
    if (state == State::Uninitialized) {
        state = State::ClothLoaded;
    }
}

bool BeltLauncherController::fire(float& target_velocity)
{
    // 装填された状態でしか射出しない
    if (state != State::ClothLoaded) return false;
    target_velocity = target_velocity_;
    state           = State::Firing;
    return true;
}

bool BeltLauncherController::load_a_cloth(float& target_angle, uint32_t now_ms)
{
    // 再装填待ちでなければ装填しない
    if (!wait_for_reload_) return false;
    // 以前ゼロ点に戻ってきたタイミングから遅れ分経ったら装填
    if (now_ms - last_initial_point_time_ms_ >= reload_delay_ms_) {
        cloth_loader_target_angle_ += reload_angle_delta_;
        target_angle     = cloth_loader_target_angle_;
        wait_for_reload_ = false;
        state            = State::ClothLoaded;
        return true;
    }
    return false;
}