#pragma once
#include <array>

struct LEDInformation {
    /*ベルト直動*/
    bool belt_initialization{};  // true:打てます！！ false:初期化未完了
    float belt_velocity{};
    /*エアー射出*/
    bool air_injection{};
    /*電圧*/
    float battery_voltage[3]{};  // 0,1:logic 2:drive
} __attribute__((__packed__));
