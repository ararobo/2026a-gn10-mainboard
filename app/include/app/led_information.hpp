#pragma once
#include <cstdint>

struct LEDInformation {
    /*ベルト直動*/
    bool belt_initialization{};
    float belt_velocity{};
    /*エアー射出*/
    bool air_injection{};  // true:injection false:not injection
    /*電圧*/
    std::array<float, 3> battery_voltage{};  // 0,1:logic 2:drive
} __attribute__((__packed__));
