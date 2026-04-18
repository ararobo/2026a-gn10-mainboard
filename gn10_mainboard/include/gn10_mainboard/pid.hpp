/**
 * @file pid.hpp
 * @author Gento Aiba (aiba-gento)
 * @brief PID制御クラス
 * @version 0.2.0
 * @date 2026-02-20
 *
 * @copyright Copyright (c) 2026 ararobo
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#pragma once

#include <algorithm>
#include <type_traits>

namespace gn10_motor {

template <typename T>
struct PIDConfig {
    T kp             = T{0};
    T ki             = T{0};
    T kd             = T{0};
    T integral_limit = T{0};
    T output_limit   = T{0};
};

template <typename T>
class PID
{
    // テンプレート引数が浮動小数点型であることを保証する (C++17)
    static_assert(std::is_floating_point_v<T>, "PID class only supports floating point types.");

public:
    explicit PID(const PIDConfig<T>& config) : config_(config) {}

    T update(T setpoint, T measurement, T dt)
    {
        // dtが0以下の場合は計算をスキップ（ゼロ除算防止）
        if (dt <= T{0}) return T{0};

        T error = setpoint - measurement;

        T p_term = config_.kp * error;

        integral_ += error * dt;

        // 積分蓄積値を制限
        integral_ = std::clamp(integral_, -config_.integral_limit, config_.integral_limit);

        T i_term = config_.ki * integral_;

        // 微分先行 (Derivative on Measurement)
        // Setpoint Kickを防ぐため、誤差(error)ではなく測定値(measurement)の微分を使用
        T derivative = (measurement - previous_measurement_) / dt;

        // 変化量(derivative)が正のとき、抑制方向へ力を加えるため -Kd を掛ける
        T d_term = -config_.kd * derivative;

        T output = p_term + i_term + d_term;

        // 前回の測定値を更新
        previous_measurement_ = measurement;

        // 出力制限
        return std::clamp(output, -config_.output_limit, config_.output_limit);
    }

    /**
     * @brief PID内部状態のリセット
     * @param current_measurement 現在の測定値（微分項のKick防止のため初期化に必要）
     */
    void reset(T current_measurement = T{0})
    {
        integral_             = T{0};
        previous_measurement_ = current_measurement;
    }

    void set_config(const PIDConfig<T>& config)
    {
        config_   = config;
        integral_ = T{0};
    }

    /**
     * @brief 内部状態（積分項など）を維持したまま設定値を変更する
     * 実行中のゲイン変更などに使用。必要に応じて積分項を新しいリミットでクランプする。
     */
    void update_config(const PIDConfig<T>& config)
    {
        config_ = config;
        // 新しいリミットに合わせて積分項をクランプし直す
        integral_ = std::clamp(integral_, -config_.integral_limit, config_.integral_limit);
    }

private:
    PIDConfig<T> config_;
    T integral_             = T{0};
    T previous_measurement_ = T{0};
};

}  // namespace gn10_motor