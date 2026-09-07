#pragma once

/**
 * @brief バケツ内部に入った雑巾を取り出すためのアーム制御用クラス
 * @note エンコーダーとモーターの正方向は異なり、ゼロ点を最高点で取る点に注意
 *
 */
class BucketArmController
{
public:
    BucketArmController(float pulley_radius, float height_max, float height_min)
        : pulley_radius_(pulley_radius), height_max_(height_max), height_min_(height_min) {};

    /**
     * @brief 高さ調節の速度比率設定
     *
     * @param velocity_ratio 0〜1の範囲で設定し、高さ調節の速度が変化
     */
    void set_height_adjustment_velocity_ratio(float velocity_ratio)
    {
        height_adjustment_velocity_ratio_ = velocity_ratio;
    }

    /**
     * @brief ハンドで雑巾を保持する際の出力値設定
     *
     * @param output_current モーターに流す電流で、保持力に比例する[A](正の値)
     */
    void set_hold_force_by_current(float output_current)
    {
        hold_current_ = output_current;
    }

    /**
     * @brief ハンドで雑巾をリリースする際の出力値設定
     *
     * @param output_current モーターに流す電流で、開放するときの力に比例する[A](正の値)
     */
    void set_release_force_by_current(float output_current)
    {
        release_current_ = output_current;
    }

    /**
     * @brief 高さ[m]を回転角[rad]に変換[
     * @note ゼロ除算を避けるため、プーリー半径が0以下では0を返す
     *
     * @param height_m 高さ[m](地面の高さを0として上昇方向を+)
     * @return float 回転角[rad](最高点(リミットスイッチ作動位置)を0として降下方向を+)
     */
    float height_to_angle(float height_m) const
    {
        if (pulley_radius_ > 0.0f) {
            return (height_max_ - height_m) / pulley_radius_;
        }
        return 0.0f;
    }

    /**
     * @brief 回転角[rad]を高さ[m]に変換
     *
     * @param angle_rad 回転角[rad](最高点(リミットスイッチ作動位置)を0として降下方向を+)
     * @return float 高さ[m](地面の高さを0として上昇方向を+)
     */
    float angle_to_height(float angle_rad) const
    {
        return height_max_ - (angle_rad * pulley_radius_);
    }

    /**
     * @brief 高さ調整用モーター出力を計算
     *
     * @param up 上昇
     * @param down 降下
     * @param angle_rad 回転角[rad](最高点(リミットスイッチ作動位置)を0として降下方向を+)
     * @return float 高さ調整用モーター出力[ratio](上昇方向を+、降下方向を-)
     */
    float height_motor_output(bool up, bool down, float angle_rad) const;

    /**
     * @brief ハンド保持用モーターの出力を計算
     *
     * @param hold 保持するかどうか
     * @return float ハンドのモーター出力(保持を+、開放を-)
     */
    float hold_motor_output(bool hold) const;

private:
    float pulley_radius_{};
    float height_max_{};
    float height_min_{};
    float height_adjustment_velocity_ratio_{};
    float hold_current_{};
    float release_current_{};
};