#include "wiznet_ether/robot_ethernet.hpp"

#include "gpio.h"
#include "wiznet_ether/ethernet_config.hpp"
#include "wiznet_ether/serial_printf.hpp"
#include "wiznet_ether/socket.hpp"
#include "wiznet_ether/w5500_spi.hpp"

bool RobotEthernet::init()
{
    if (W5500Init()) {
    } else {
        HAL_GPIO_WritePin(LED_RAD_GPIO_Port, LED_RAD_Pin, GPIO_PIN_SET);
        return false;
    }

    wizchip_setnetinfo(&ethernet_config::main_board::netInfo);
    // ネットワーク情報の確認
    wiz_NetInfo tmpNetInfo;
    wizchip_getnetinfo(&tmpNetInfo);
    log_printf(
        LOG_INFO,
        "MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        tmpNetInfo.mac[0],
        tmpNetInfo.mac[1],
        tmpNetInfo.mac[2],
        tmpNetInfo.mac[3],
        tmpNetInfo.mac[4],
        tmpNetInfo.mac[5]
    );
    log_printf(
        LOG_INFO,
        "IP: %d.%d.%d.%d\n",
        tmpNetInfo.ip[0],
        tmpNetInfo.ip[1],
        tmpNetInfo.ip[2],
        tmpNetInfo.ip[3]
    );

    setRCR(1);
    setRTR(100);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    socket(
        socket_operation, Sn_MR_UDP, ethernet_config::main_board::port_operation, SF_IO_NONBLOCK
    );
    setSn_CR(socket_operation, Sn_CR_RECV);
    if (getSn_SR(socket_operation) == SOCK_UDP) {
    } else {
        return false;
    }
    return true;
}

void RobotEthernet::send_operation_data(operation_data_t data)
{
    data.header          = operation_data_header;
    operation_union.data = data;
    sendto(
        socket_operation,
        operation_union.code,
        sizeof(operation_data_union_t),
        ethernet_config::pc::ip,
        ethernet_config::pc::port_operation
    );
}

bool RobotEthernet::receive_operation_data(operation_data_t* data)
{
    int32_t ret = recvfrom(
        socket_operation,
        operation_union.code,
        sizeof(operation_data_union_t),
        ethernet_config::pc::ip,
        &ethernet_config::pc::port_operation
    );
    if (ret == sizeof(operation_data_union_t) &&
        operation_union.data.header == operation_data_header) {
        *data = operation_union.data;
        return true;
    }
    return false;
}