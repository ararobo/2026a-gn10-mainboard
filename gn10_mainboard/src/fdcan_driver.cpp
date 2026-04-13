#include "gn10_mainboard/fdcan_driver.hpp"

namespace gn10_can {
namespace drivers {

bool FDCANDriver::init()
{
    FDCAN_FilterTypeDef filter;
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0x000;
    filter.FilterID2    = 0x000;

    if (HAL_FDCAN_ConfigFilter(hfdcan_, &filter) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_Start(hfdcan_) != HAL_OK) {
        return false;
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        return false;
    }
    return true;
}

bool FDCANDriver::send(const FDCANFrame& frame)
{
    FDCAN_TxHeaderTypeDef tx_header;
    if (frame.is_extended) {
        tx_header.IdType = FDCAN_EXTENDED_ID;
    } else {
        tx_header.IdType = FDCAN_STANDARD_ID;
    }
    tx_header.Identifier          = frame.id;
    tx_header.TxFrameType         = FDCAN_DATA_FRAME;
    tx_header.DataLength          = frame.dlc;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch       = FDCAN_BRS_OFF;
    tx_header.FDFormat            = FDCAN_FD_CAN;
    tx_header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker       = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(
            hfdcan_, &tx_header, const_cast<uint8_t*>(frame.data.data())
        ) != HAL_OK) {
        return false;
    }
    return true;
}

bool FDCANDriver::receive(FDCANFrame& out_frame)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];

    if (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return false;
    }

    out_frame.id          = rx_header.Identifier;
    out_frame.is_extended = (rx_header.IdType == FDCAN_EXTENDED_ID);

    // --- ここから修正 ---
    // DLCコードを実際のバイト数に変換するヘルパー関数（HALに用意されています）
    uint32_t byte_len = 0;

    // HALによっては FDCAN_ConvertDataLength() という関数が使えます
    // もしくは、以下のような単純な switch/if 文で変換が必要です
    if (rx_header.DataLength <= FDCAN_DLC_BYTES_8) {
        byte_len = rx_header.DataLength;
    } else {
        // FDCAN特有のDLC変換（例: 0x9 -> 12, 0xA -> 16...）
        // HALの定義値を使ってバイト数を取得
        byte_len = convert_dlc_to_bytes(rx_header.DataLength);
    }

    out_frame.dlc = static_cast<uint8_t>(byte_len);

    // パケットサイズが16バイト（Jetsonのパディング後）であっても、
    // 自作構造体のサイズ（14バイト）を超えないようにガードをかける
    uint8_t copy_len = std::min<uint8_t>(out_frame.dlc, 64);

    for (uint8_t i = 0; i < copy_len; ++i) {
        out_frame.data[i] = rx_data[i];
    }
    // -------------------

    return true;
}

// 補足：DLCを変換する関数の例
uint32_t FDCANDriver::convert_dlc_to_bytes(uint32_t dlc)
{
    switch (dlc) {
        case FDCAN_DLC_BYTES_12:
            return 12;
        case FDCAN_DLC_BYTES_16:
            return 16;
        case FDCAN_DLC_BYTES_20:
            return 20;
        case FDCAN_DLC_BYTES_24:
            return 24;
        case FDCAN_DLC_BYTES_32:
            return 32;
        case FDCAN_DLC_BYTES_48:
            return 48;
        case FDCAN_DLC_BYTES_64:
            return 64;
        default:
            return dlc;  // 8以下はそのまま
    }
}

}  // namespace drivers
}  // namespace gn10_can
