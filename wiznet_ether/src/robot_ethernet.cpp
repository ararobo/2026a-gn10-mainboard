#include "wiznet_ether/robot_ethernet.hpp"

#include "gpio.h"
#include "wiznet_ether/ethernet_config.hpp"
#include "wiznet_ether/socket.hpp"
#include "wiznet_ether/w5500_spi.hpp"

bool RobotEthernet::init()
{
    if (W5500Init()) {
    } else {
        return false;
    }

    wizchip_setnetinfo(&ethernet_config::main_board::netInfo);
    // ネットワーク情報の確認
    wiz_NetInfo tmpNetInfo;
    wizchip_getnetinfo(&tmpNetInfo);

    setRCR(1);
    setRTR(100);

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