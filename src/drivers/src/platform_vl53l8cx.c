/**
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdlib.h>
#include <string.h>
// #include "platform.h"
/* Include the VL53L8CX platform interface so C types like VL53L8CX_Platform
 * and the VL53L8CX_* function prototypes are visible to this C++ file. We
 * use the relative path into the drivers interface directory. */
#include "vl53l8cx_api.h"
#include "FreeRTOS.h"
#include "debug.h"
#include "deck.h"
#include "deck_spi.h"
#include "deck_spi3.h"
#include "platform_vl53l8cx.h"
#include "stm32f4xx_spi.h"
#include "task.h"

// Provide two alternatives: the original mbed implementation when USE_MBED
// is defined, otherwise fallback to lightweight GCC-friendly stubs that
// allow compilation on a normal host toolchain. The stubs are simple and
// intended as placeholders; replace thinit_IOem with real hardware bindings as
// needed (e.g., spidev, sysfs GPIO, wiringPi, libgpiod, etc.).

#ifndef USE_MBED

// Define simple integer constants for pin names used in the original code.
#define PA_7 0
#define PA_6 1
#define PB_3 2
#define PB_12 3
#define PB_6 4
#define PC_7 5

#endif  // USE_MBED

static deckPin_t CS0;
static deckPin_t CS1;
static deckPin_t CS2;

// This function is called once at startup
// It initializes the SPI and the GPIOs
void init_IO()
{
    // Spi.format(8, 3);
    // Spi.frequency(2500000);
    CS0 = DECK_GPIO_IO4;
    CS1 = DECK_GPIO_IO3;
    CS2 = DECK_GPIO_IO2;
    pinMode(CS0, OUTPUT);
    pinMode(CS1, OUTPUT);
    pinMode(CS2, INPUT);
    digitalWrite(CS0, HIGH);
    digitalWrite(CS1, HIGH);
    spiBegin();
}

void cs_low(deckPin_t cs_pin) { digitalWrite(cs_pin, LOW); }
void cs_high(deckPin_t cs_pin) { digitalWrite(cs_pin, HIGH); }

volatile uint16_t BckDev = 0xFFFF;

// 呼び出しもとでspiBeginTransaction/EndTransactionで囲むこと
uint16_t Ser_IT()
{
    static uint16_t Intr;
    uint8_t high_byte, low_byte;

    if (digitalRead(CS2) == LOW)
    {
        // DEBUG_PRINT("Ser_IT: CS2 LOW\n");
        return 0;
    }
    uint8_t read_addr_high = (uint8_t)((BckDev >> 8) & 0xFF);
    uint8_t read_addr_low = (uint8_t)(BckDev & 0xFF);
    cs_low(CS0);
    spiExchange(1, &read_addr_high, &high_byte);
    spiExchange(1, &read_addr_low, &low_byte);
    Intr = (uint16_t)high_byte << 8;
    Intr |= (uint16_t)low_byte;
    cs_high(CS0);
    return Intr;
}


uint8_t VL53L8CX_RdByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value)
{
    unsigned char rD[3];
    uint8_t status = 255;
    /* Need to be implemented by customer. This function returns 0 if OK */
    cs_low(CS1);
    // dititalwrite(DECK_GPIO_IO4, LOW);
    // Read: 先頭ビットを読み取りビットとして0に設定
    uint8_t read_addr_high = (uint8_t)((RegisterAdress >> 8) & 0x7F);
    uint8_t read_addr_low = (uint8_t)(RegisterAdress & 0xFF);

    // アドレス送信、0x00はdummy
    spiExchange(1, &read_addr_high, &rD[0]);
    spiExchange(1, &read_addr_low, &rD[1]);
    spiExchange(1, 0x00, &rD[2]);

    cs_high(CS1);
    *p_value = rD[2];
    status = 0;
    return status;
}

// 呼び出しもとでspiBeginTransaction/EndTransactionで囲むこと
uint8_t VL53L8CX_WrByte(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value)
{
    unsigned char rD[3];
    uint8_t status = 255;

    uint8_t read_addr_high = (uint8_t)((RegisterAdress >> 8) | 0x80);
    uint8_t read_addr_low = (uint8_t)(RegisterAdress & 0xFF);

    cs_low(CS1);
    spiExchange(1, &read_addr_high, &rD[0]);
    spiExchange(1, &read_addr_low, &rD[1]);
    spiExchange(1, &value, &rD[2]);
    cs_high(CS1);

    status = 0;
    return status;
}

// 呼び出しもとでspiBeginTransaction/EndTransactionで囲むこと
// uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
// {
//     unsigned char rD;
//     uint8_t status = 0;

//     Sel_Dev(p_platform->address);

//     // DEBUG_PRINT("VL53L8CX_WrMulti: offset=%lu/%lu\n", offset, size);

//     uint8_t read_addr_high = (uint8_t)((RegisterAdress >> 8) | 0x80);
//     uint8_t read_addr_low = (uint8_t)(RegisterAdress & 0xFF);

//     // チャンク毎に宛先デバイスを再選択（トランザクション再取得後の安全性向上）
//     Sel_Dev(p_platform->address);

//     // 完全なフレームとして1チャンク送信
//     cs_low(CS1);
//     spiExchange(1, &read_addr_high, &rD);
//     spiExchange(1, &read_addr_low, &rD);
//     for (uint32_t n = 0; n < size; n++)
//     {
//         spiExchange(1, &p_values[n], &rD);
//     }
//     cs_high(CS1);

//     return status;
// }

// 呼び出しもとでspiBeginTransaction/EndTransactionで囲むこと
uint8_t VL53L8CX_WrMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
{
    uint32_t offset = 0;  // 書き込み済みのバイト数
    unsigned char rD;     // ダミー変数、spiExchangeの引数として必要
    uint8_t status = 0;
    const uint32_t CHUNK_SIZE = 0x8000;  // 1回のSPIでまとめて書き込む最大バイト数

    // 64バイトごとのchunkで処理
    while (offset < size)
    {
        // DEBUG_PRINT("VL53L8CX_WrMulti: offset=%lu/%lu\n", offset, size);
        uint32_t chunk_len = (size - offset) > CHUNK_SIZE ? CHUNK_SIZE : (size - offset);
        uint16_t chunk_addr = RegisterAdress + offset;
        uint8_t read_addr_high = (uint8_t)((chunk_addr >> 8) | 0x80);
        uint8_t read_addr_low = (uint8_t)(chunk_addr & 0xFF);


        // 完全なフレームとして 1chunk 送信
        cs_low(CS1);
        spiExchange(1, &read_addr_high, &rD);
        spiExchange(1, &read_addr_low, &rD);
        for (uint32_t n = 0; n < chunk_len; n++)
        {
            spiExchange(1, &p_values[offset + n], &rD);
        }
        cs_high(CS1);

        offset += chunk_len;

        // chunkごとにtaskにdelayを入れる
        vTaskDelay_for_spi_pause(3);
    }

    // WrMultiの後に書き込み内容を読み取って確認するコード（デバッグ用）
    offset = 0;  // 書き込み済みのバイト数
    while (offset < size)
    {
        uint32_t chunk_len = (size - offset) > CHUNK_SIZE ? CHUNK_SIZE : (size - offset);
        uint16_t chunk_addr = RegisterAdress + offset;
        uint8_t read_addr_high = (uint8_t)((chunk_addr >> 8) & 0x7F);
        uint8_t read_addr_low = (uint8_t)(chunk_addr & 0xFF);

        cs_low(CS1);
        spiExchange(1, &read_addr_high, &rD);
        spiExchange(1, &read_addr_low, &rD);
        for (uint32_t n = 0; n < chunk_len; n++)
        {
            spiExchange(1, 0x00, &rD);  // ダミー送信
            if (rD != p_values[offset + n])
            {
                // DEBUG_PRINT("WrMulti failed :offset %lu (expected %02X, got %02X)\n", offset + n, p_values[offset + n],
                //             rD);
            }
        }
        cs_high(CS1);
        offset += chunk_len;
        vTaskDelay_for_spi_pause(3);
    }

    return status;
}

uint8_t VL53L8CX_WrMultiFW(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size,
                           uint16_t page)
{
    uint32_t offset = 0;
    unsigned char rD;
    uint8_t status = 0;
    const uint32_t CHUNK_SIZE = 0x8000;
    while (offset < size)  // 64バイトごとに処理
    {
        uint32_t chunk_len = (size - offset) > CHUNK_SIZE ? CHUNK_SIZE : (size - offset);
        uint16_t chunk_addr = RegisterAdress + offset;
        uint8_t read_addr_high = (uint8_t)((chunk_addr >> 8) | 0x80);
        uint8_t read_addr_low = (uint8_t)(chunk_addr & 0xFF);

        // 完全なフレームとして1チャンク送信
        cs_low(CS1);
        spiExchange(1, &read_addr_high, &rD);
        spiExchange(1, &read_addr_low, &rD);
        for (uint32_t n = 0; n < chunk_len; n++)
        {
            spiExchange(1, &p_values[offset + n], &rD);
        }
        cs_high(CS1);

        offset += chunk_len;

        // chunkごとにtaskにdelayを入れる
        vTaskDelay_for_spi_pause(5);
        // VL53L8CX_WrByte(p_platform, 0x7fff, page);
    }

    // WrMultiの後に書き込み内容を読み取って確認するコード（デバッグ用）
    offset = 0;  // 書き込み済みのバイト数
    vTaskDelay_for_spi_pause(5);
    while (offset < size)
    {
        uint32_t chunk_len = (size - offset) > CHUNK_SIZE ? CHUNK_SIZE : (size - offset);
        uint16_t chunk_addr = RegisterAdress + offset;
        uint8_t read_addr_high = (uint8_t)((chunk_addr >> 8) & 0x7F);
        uint8_t read_addr_low = (uint8_t)(chunk_addr & 0xFF);

        cs_low(CS1);
        spiExchange(1, &read_addr_high, &rD);
        spiExchange(1, &read_addr_low, &rD);
        for (uint32_t n = 0; n < chunk_len; n++)
        {
            spiExchange(1, 0x00, &rD);  // ダミー送信
            if (rD != p_values[offset + n])
            {
                // DEBUG_PRINT("FW fail at %lu ( exp %02X, got %02X)\n", offset + n, p_values[offset + n], rD);
            }
        }
        cs_high(CS1);
        offset += chunk_len;
        vTaskDelay_for_spi_pause(3);
    }

    return status;
}

// 呼び出しもとでspiBeginTransaction/EndTransactionで囲むこと
// this function is called by API functions
// status is updated with I2C error status
uint8_t VL53L8CX_RdMulti(VL53L8CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size)
{
    uint32_t n;
    unsigned char rD;
    uint8_t status = 255;
    uint8_t read_addr_high = (uint8_t)((RegisterAdress >> 8) & 0x7F);
    uint8_t read_addr_low = (uint8_t)(RegisterAdress & 0xFF);

    cs_low(CS1);
    spiExchange(1, &read_addr_high, &rD);
    spiExchange(1, &read_addr_low, &rD);
    for (n = 0x0; n < size; n++)
    {
        spiExchange(1, 0x00, &p_values[n]);
    }
    cs_high(CS1);
    status = 0;
    return status;
}

uint8_t VL53L8CX_Reset_Sensor(VL53L8CX_Platform *p_platform)
{
    uint8_t status = 0;

    /* (Optional) Need to be implemented by customer. This function returns 0 if OK */

    /* Set pin LPN to LOW */
    /* Set pin AVDD to LOW */
    /* Set pin VDDIO  to LOW */
    /* Set pin CORE_1V8 to LOW */
    VL53L8CX_WaitMs(p_platform, 100);

    /* Set pin LPN to HIGH */
    /* Set pin AVDD to HIGH */
    /* Set pin VDDIO to HIGH */
    /* Set pin CORE_1V8 to HIGH */
    VL53L8CX_WaitMs(p_platform, 100);

    return status;
}

void VL53L8CX_SwapBuffer(uint8_t *buffer, uint16_t size)
{
    uint32_t i, tmp;

    /* Example of possible implementation using <string.h> */
    for (i = 0; i + 3 < size; i = i + 4)
    {
        tmp = (((uint32_t)buffer[i] << 24) | ((uint32_t)buffer[i + 1] << 16) | ((uint32_t)buffer[i + 2] << 8) |
               ((uint32_t)buffer[i + 3]));

        memcpy(&(buffer[i]), &tmp, 4);
    }
}

uint8_t VL53L8CX_WaitMs(VL53L8CX_Platform *p_platform, uint32_t TimeMs)
{
    uint8_t status = 255;

    /* Need to be implemented by customer. This function returns 0 if OK */
    vTaskDelay(pdMS_TO_TICKS(TimeMs));
    status = 0;
    return status;
}

// spi transaction を中断してCPUを解放するバージョン
uint8_t VL53L8CX_WaitMs_spi_pause(VL53L8CX_Platform *p_platform, uint32_t TimeMs)
{
    uint8_t status = 255;
    vTaskDelay_for_spi_pause(TimeMs);
    status = 0;

    return status;
}

void cpu_release_for_spi_pause(uint8_t *CS_PIN_STATE)
{
    CS_PIN_STATE[0] = digitalRead(CS0);
    CS_PIN_STATE[1] = digitalRead(CS1);
    if (CS_PIN_STATE[0] == LOW)
    {
        cs_high(CS0);
    }
    if (CS_PIN_STATE[1] == LOW)
    {
        cs_high(CS1);
    }
}

void cpu_reacquire_after_spi_pause(uint8_t *CS_PIN_STATE)
{
    if (CS_PIN_STATE[0] == LOW)
    {
        cs_low(CS0);
    }
    if (CS_PIN_STATE[1] == LOW)
    {
        cs_low(CS1);
    }
}

void vTaskDelay_for_spi_pause(uint32_t TimeMs)
{
    spiEndTransaction();
    // uint8_t CS_PIN_STATE[2] = {};
    // cpu_release_for_spi_pause(CS_PIN_STATE);
    vTaskDelay(pdMS_TO_TICKS(TimeMs));
    // cpu_reacquire_after_spi_pause(CS_PIN_STATE);
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
}

// /* ===== newlib I/O stubs for vTaskList() support ===== */
// int _write(int file, const char *ptr, int len)
// {
//     (void)file;
//     for (int i = 0; i < len; i++)
//     {
//         DEBUG_PRINT("%c", ptr[i]);
//     }
//     return len;
// }

// int _read(int file, char *ptr, int len)
// {
//     (void)file;
//     (void)ptr;
//     (void)len;
//     return 0;
// }

// int _close(int file)
// {
//     (void)file;
//     return 0;
// }

// int _lseek(int file, int ptr, int dir)
// {
//     (void)file;
//     (void)ptr;
//     (void)dir;
//     return 0;
// }

// int _fstat(int file, void *st)
// {
//     (void)file;
//     (void)st;
//     return 0;
// }

// int _isatty(int file)
// {
//     (void)file;
//     return 1;
// }

// void *_sbrk(int incr)
// {
//     (void)incr;
//     return (void *)-1;
// }