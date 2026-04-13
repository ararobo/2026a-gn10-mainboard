/**
 * @file driver_stm32_fdcan.hpp
 * @author Gento Aiba (aiba-gento)
 * @brief STM32 FDCANのドライバ具体化クラスのヘッダファイル
 * @version 0.1.0
 * @date 2026-01-11
 *
 * @copyright Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "gn10_can/drivers/fdcan_driver_interface.hpp"
#include "main.h"

namespace gn10_can {
namespace drivers {

class FDCANDriver : public IFDCANDriver
{
public:
    FDCANDriver(FDCAN_HandleTypeDef* hfdcan) : hfdcan_(hfdcan) {}

    bool init();
    bool send(const FDCANFrame& frame) override;
    bool receive(FDCANFrame& out_frame) override;

private:
    FDCAN_HandleTypeDef* hfdcan_;
};
}  // namespace drivers
}  // namespace gn10_can
