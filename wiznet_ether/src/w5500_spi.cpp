#include "wiznet_ether/w5500_spi.hpp"

#include "gpio.h"
#include "spi.h"
#include "stm32g474xx.h"
#include "stm32g4xx_hal.h"
#include "wiznet_ether/wizchip_conf.hpp"

// DMA転送完了フラグ
volatile bool dma_rx_complete = false;
volatile bool dma_tx_complete = false;

// DMA完了コールバック関数
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi->Instance == SPI1) {
        dma_rx_complete = true;
        dma_tx_complete = true;
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi->Instance == SPI1) {
        dma_rx_complete = true;
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    if (hspi->Instance == SPI1) {
        dma_tx_complete = true;
    }
}

/**
 * @brief １バイトのデータを送受信する（DMA）
 *
 * @param data 送信するデータ
 * @return uint8_t 受信したデータ
 */
uint8_t SPIReadWrite(uint8_t data)
{
    uint8_t rxData;
    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(&hspi1, &data, &rxData, 1, 100);  // 送信と受信を同時に行う
    if (status == HAL_TIMEOUT) {
    }
    return rxData;
}
/**
 * @brief W5500のCSを選択する
 *
 */
void wizchip_select(void) {}

/**
 * @brief W5500のCSを解除する
 *
 */
void wizchip_deselect(void) {}

/**
 * @brief W5500にデータを書き込む
 *
 * @param wb 書き込むデータ
 */
void wizchip_write(uint8_t wb)
{
    SPIReadWrite(wb);  // 1バイトのデータを送信
}

/**
 * @brief W5500からデータを読み込む
 *
 * @return uint8_t 読み込んだデータ
 */
uint8_t wizchip_read()
{
    return SPIReadWrite(0x00);  // 1バイトのデータを受信
}

/**
 * @brief W5500からデータを読み込む
 *
 * @param pBuf 読み込んだデータを格納するバッファ
 * @param len 読み込むデータの長さ
 */
void wizchip_readburst(uint8_t* pBuf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)  // 指定した長さだけデータを読み込む
    {
        *pBuf = SPIReadWrite(0x00);  // データを受信
        pBuf++;                      // バッファを次に進める
    }
}

/**
 * @brief W5500にデータを書き込む
 *
 * @param pBuf 書き込むデータを格納したバッファ
 * @param len 書き込むデータの長さ
 */
void wizchip_writeburst(uint8_t* pBuf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)  // 指定した長さだけデータを書き込む
    {
        SPIReadWrite(*pBuf);  // データを送信
        pBuf++;               // バッファを次に進める
    }
}

/**
 * @brief W5500のIOを初期化する
 *
 */
void W5500IOInit()
{
    GPIO_InitTypeDef GPIO_InitStruct[2] = {0};  // GPIOの設定構造体

    __HAL_RCC_GPIOA_CLK_ENABLE();  // GPIOAのクロックを有効化

    GPIO_InitStruct[0].Pin   = SPI1_CS_Pin;           // CSピン
    GPIO_InitStruct[0].Mode  = GPIO_MODE_OUTPUT_PP;   // 出力モード
    GPIO_InitStruct[0].Pull  = GPIO_NOPULL;           // プルアップ/ダウンなし
    GPIO_InitStruct[0].Speed = GPIO_SPEED_FREQ_HIGH;  // 出力速度は高速

    HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct[0]);  // GPIOの初期化

    GPIO_InitStruct[1].Pin   = W5500_RST_Pin;         // RSTピン
    GPIO_InitStruct[1].Mode  = GPIO_MODE_OUTPUT_PP;   // 出力モード
    GPIO_InitStruct[1].Pull  = GPIO_NOPULL;           // プルアップ/ダウンなし
    GPIO_InitStruct[1].Speed = GPIO_SPEED_FREQ_HIGH;  // 出力速度は高速

    HAL_GPIO_Init(W5500_RST_GPIO_Port, &GPIO_InitStruct[1]);  // GPIOの初期化
}

bool W5500Init()
{
    uint8_t tmp;
    uint8_t memsize[2][8] = {
        {2, 2, 2, 2, 2, 2, 2, 2},
        {2, 2, 2, 2, 2, 2, 2, 2}
    };

    // W5500IOInit();
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);  // CS HIGH is default

    // send reset signal
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_RESET);  // RST LOW
    tmp = 0xFF;
    while (tmp--)  // 一定時間待つ
        ;
    HAL_GPIO_WritePin(W5500_RST_GPIO_Port, W5500_RST_Pin, GPIO_PIN_SET);  // RST HIGH

    // ライブラリへ自作関数の紐付け
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);  // CSのコールバック関数を登録
    reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);      // SPIのコールバック関数を登録
    reg_wizchip_spiburst_cbfunc(
        wizchip_readburst, wizchip_writeburst
    );  // SPIバーストのコールバック関数を登録

    if (ctlwizchip(CW_INIT_WIZCHIP, (void*)memsize) == -1)  // W5500の初期化
    {
        return false;
    } else {
        return true;
    }
}