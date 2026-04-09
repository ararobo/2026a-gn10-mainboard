#pragma once
// #define ENABLE_PID_GAIN_UDP
// #define ENABLE_CONTROLLER_BYPASS_UDP
#include "wiznet_ether/robot_data_config.hpp"

class RobotEthernet
{
private:
    uint8_t socket_operation = 0;  // 操作用ソケット

    union operation_data_union_t {
        operation_data_t data;                   // 操作データ
        uint8_t code[sizeof(operation_data_t)];  // 送信バイト配列
    } __attribute__((__packed__)) operation_union;

public:
    bool init();
    void send_operation_data(operation_data_t data);
    bool receive_operation_data(operation_data_t* data);
};
