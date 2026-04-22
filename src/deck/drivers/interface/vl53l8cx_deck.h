#pragma once
#include <stdbool.h>
#include <stdint.h>

#include "deck_digital.h"
#include "deck_spi.h"
#include "param.h"
#include "vl53l8cx_api.h"

#ifndef vl53l8cx_DEFAULT_RATE_HZ
#define vl53l8cx_DEFAULT_RATE_HZ 10
#endif
#ifndef vl53l8cx_POLL_PERIOD_MS
#define vl53l8cx_POLL_PERIOD_MS 20
#endif

#ifndef OUTPUT
#define OUTPUT 1
#endif
#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif

/* SPI helpers (override if you use custom names) */
#ifndef vl53l8cx_SPI_ACQUIRE
#define vl53l8cx_SPI_ACQUIRE() ((void)0)
#endif
#ifndef vl53l8cx_SPI_RELEASE
#define vl53l8cx_SPI_RELEASE() ((void)0)
#endif
#ifndef vl53l8cx_SPI_START
#define vl53l8cx_SPI_START(cfg) \
    do                          \
    {                           \
        (void)(cfg);            \
        spiBegin();             \
        spiBeginTransaction(8); \
    } while (0)
#endif
#ifndef vl53l8cx_SPI_SEND
#define vl53l8cx_SPI_SEND(n, buf)                             \
    do                                                        \
    {                                                         \
        (void)spiExchange((n), (const uint8_t *)(buf), NULL); \
    } while (0)
#endif
#ifndef vl53l8cx_SPI_RECV
#define vl53l8cx_SPI_RECV(n, buf)                       \
    do                                                  \
    {                                                   \
        (void)spiExchange((n), NULL, (uint8_t *)(buf)); \
    } while (0)
#endif
#ifndef vl53l8cx_SPI_SEND_WRITE
#define vl53l8cx_SPI_SEND_WRITE(n, buf)                       \
    do                                                        \
    {                                                         \
        (void)spiExchange((n), (const uint8_t *)(buf), NULL); \
    } while (0)
#endif
#ifndef vl53l8cx_SPI_RECV_WRITE
#define vl53l8cx_SPI_RECV_WRITE(n, buf)                 \
    do                                                  \
    {                                                   \
        (void)spiExchange((n), NULL, (uint8_t *)(buf)); \
    } while (0)
#endif
#ifndef vl53l8cx_SPI_SEND_READ
#define vl53l8cx_SPI_SEND_READ(n, buf)                        \
    do                                                        \
    {                                                         \
        (void)spiExchange((n), (const uint8_t *)(buf), NULL); \
    } while (0)
#endif
#ifndef vl53l8cx_SPI_RECV_READ
#define vl53l8cx_SPI_RECV_READ(n, buf)                  \
    do                                                  \
    {                                                   \
        (void)spiExchange((n), NULL, (uint8_t *)(buf)); \
    } while (0)
#endif

#ifndef vl53l8cx_GPIO_INIT_OUTPUT
#define vl53l8cx_GPIO_INIT_OUTPUT(pin) pinMode((pin), OUTPUT)
#endif
#ifndef vl53l8cx_GPIO_WRITE
#define vl53l8cx_GPIO_WRITE(pin, lvl) digitalWrite((pin), (lvl))
#endif

/* CS lines provided in the .c (fill with real deckPin_t pins) */
extern const deckPin_t g_vl53l8cx_cs[vl53l8cx_NUM_SENSORS];

/* Public helper API (optional) */
#ifdef __cplusplus
extern "C"
{
#endif
    bool vl53l8cxIsRunning(void);
    uint16_t vl53l8cxGetLastMm(int index);
#ifdef __cplusplus
}
#endif