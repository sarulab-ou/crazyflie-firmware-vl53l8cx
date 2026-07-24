#include "vl53l8cx_deck.h"

#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "debug.h"
#include "deck.h"
#include "led.h"
#include "log.h"
#include "param.h"
#include "static_mem.h"
#include "system.h"
#include "task.h"
#include "vl53l8cx_api.h"
/* Deck SPI and GPIO APIs */
#include "deck_constants.h"
#include "deck_digital.h"
#include "deck_spi.h"
#include "stm32f4xx_spi.h" /* For SPI_BaudRatePrescaler_* definitions */

/* ===== Memory/RAM strategy toggles =====
 * vl53l8cx_USE_CCM: place large non-DMA buffers in CCM (64KB fast SRAM, non-DMA) to free main SRAM.
 * vl53l8cx_DEFAULT_RATE_HZ: per-sensor ranging frequency (before multiplexing).
 */
#ifndef vl53l8cx_USE_CCM
#define vl53l8cx_USE_CCM 1
#endif
#ifndef vl53l8cx_DEFAULT_RATE_HZ
#define vl53l8cx_DEFAULT_RATE_HZ 10
#endif
// #include "vl53l8cx_arena.h"

/* Extern-only declarations of the ULD blobs. Must be defined exactly once elsewhere. */
#include "platform_vl53l8cx.h"
#include "vl53l8cx_buffers.h"

/* ===== Driver state (ultra-low RAM) ===== */
// static volatile uint8_t g_running = 0;
// static uint8_t g_rate_hz = vl53l8cx_DEFAULT_RATE_HZ;
// static uint32_t g_tick = 0;


static TaskHandle_t g_task = NULL;

/* Per-sensor average of the 16 zone distances [mm]. Updated in Gget_Ranging() (system.c). */
float vl53l8cxToFAvg[vl53l8cx_NUM_SENSORS] = {0};
/* Per-sensor raw distance [mm] and target_status of each of the 16 zones. Updated in Gget_Ranging() (system.c). */
int16_t vl53l8cxToFDist[vl53l8cx_NUM_SENSORS][16] = {{0}};
uint8_t vl53l8cxToFStatus[vl53l8cx_NUM_SENSORS][16] = {{0}};
// static uint8_t  g_initOk[vl53l8cx_NUM_SENSORS] = {0};

/* Build-time switch to bypass ULD init for isolation tests (0 = skip, 1 = call init) */
#ifndef vl53l8cx_CALL_INIT
#define vl53l8cx_CALL_INIT 1
#endif

/* ONE shared configuration + ONE shared results */
// NO_DMA_CCM_SAFE_ZERO_INIT static VL53L8CX_Configuration g_dev;
#ifdef vl53l8cx_USE_CCM
__attribute__((section(".ccmram")))
#endif
// static VL53L8CX_ResultsData g_res;

/* public last distances */
// static uint16_t g_ranges_mm[vl53l8cx_NUM_SENSORS];

// #ifdef vl53l8cx_USE_CCM
// __attribute__((section(".ccmram")))
// #endif


// uint8_t callbacked = 0;

int Init_Sensor(uint16_t DevAddr, uint8_t Frequency)
{
    uint8_t status, isAlive;

    MDev[DevAddr].platform.address = DevAddr;

    status = vl53l8cx_is_alive(&MDev[DevAddr], &isAlive);
    if (!isAlive || status)
    {
        DEBUG_PRINT("vl53l8cx_is_alive failed, status %u[%d]\n", status, DevAddr);
        return (0);
    }
    status = vl53l8cx_init(&MDev[DevAddr]);
    if (status)
    {
        DEBUG_PRINT("VL53L8CX ULD Loading failed_init[%d]. status is %u\n", DevAddr, status);
        return (0);
    }
    // DEBUG_PRINT("VL53L8CX ULD ready ! (Version : %s)[%d]\n", VL53L8CX_API_REVISION, DevAddr);

    status = vl53l8cx_set_ranging_frequency_hz(&MDev[DevAddr], Frequency);
    if (status)
    {
        DEBUG_PRINT("vl53l8cx_set_ranging_frequency_hz failed, status %u[%d]\n", status, DevAddr);
        return (0);
    }
    status = vl53l8cx_start_ranging(&MDev[DevAddr]);
    if (status)
    {
        DEBUG_PRINT("vl53l8cx_start_ranging failed, status %u[%d]\n", status, DevAddr);
        return (0);
    }
    return (1);
}

void Start_Ranging(uint16_t DevAddr)
{
    uint8_t status;

    MDev[DevAddr].platform.address = DevAddr;
    status = vl53l8cx_start_ranging(&MDev[DevAddr]);
}

// uint8_t DevAddr[11];
uint8_t ReStart[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t NumRdy[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};



static void vl53l8cxTask(void* arg)
{
    (void)arg;
    systemWaitStart();
    vTaskDelete(NULL);
}


/* ===== Params / Logs ===== */
// PARAM_GROUP_START(vl53l8cx)
// PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, enable, &g_running, onEnableUpdated)
// PARAM_ADD(PARAM_UINT8, rate_hz, &g_rate_hz)
// PARAM_GROUP_STOP(vl53l8cx)

// LOG_GROUP_START(vl53l8cx)
// LOG_ADD(LOG_UINT32, tick, &g_tick)
// LOG_ADD(LOG_UINT16, s0, &g_ranges_mm[0])
// // LOG_ADD(LOG_UINT16, s1,  &g_ranges_mm[1])
// // LOG_ADD(LOG_UINT16, s2,  &g_ranges_mm[2])
// // LOG_ADD(LOG_UINT16, s3,  &g_ranges_mm[3])
// // LOG_ADD(LOG_UINT16, s4,  &g_ranges_mm[4])
// // LOG_ADD(LOG_UINT16, s5,  &g_ranges_mm[5])
// // LOG_ADD(LOG_UINT16, s6,  &g_ranges_mm[6])
// // LOG_ADD(LOG_UINT16, s7,  &g_ranges_mm[7])
// // LOG_ADD(LOG_UINT16, s8,  &g_ranges_mm[8])
// // LOG_ADD(LOG_UINT16, s9,  &g_ranges_mm[9])
// // LOG_ADD(LOG_UINT16, s10, &g_ranges_mm[10])
// LOG_GROUP_STOP(vl53l8cx)

/* ===== Deck glue ===== */
static void vl53l8cxInit(DeckInfo* info)
{
    (void)info;
    if (g_task == NULL)
    {
        xTaskCreate(vl53l8cxTask, "vl53l8cx", 512, NULL, tskIDLE_PRIORITY + 2, &g_task);
    }
}

// bool vl53l8cxIsRunning(void) { return g_running != 0; }
// uint16_t vl53l8cxGetLastMm(int index) { if (index < 0 || index >= vl53l8cx_NUM_SENSORS) return 0; return
// g_ranges_mm[index];
// }

static const DeckDriver bcVL53L8CX11 = {.name = "bcVL53L8CX11", .init = vl53l8cxInit};
DECK_DRIVER(bcVL53L8CX11);

/* ===== Logs: per-sensor averaged ToF distance [mm] ===== */
LOG_GROUP_START(vl53l8cx)
/**
 * @brief Sensor 0: average of the 16 zone distances [mm]
 */
LOG_ADD(LOG_FLOAT, s0, &vl53l8cxToFAvg[0])
/**
 * @brief Sensor 1: average of the 16 zone distances [mm]
 */
LOG_ADD(LOG_FLOAT, s1, &vl53l8cxToFAvg[1])
/**
 * @brief Sensor 2: average of the 16 zone distances [mm]
 */
LOG_ADD(LOG_FLOAT, s2, &vl53l8cxToFAvg[2])
/**
 * @brief Sensor 3: average of the 16 zone distances [mm]
 */
LOG_ADD(LOG_FLOAT, s3, &vl53l8cxToFAvg[3])
/**
 * @brief Sensor 4: average of the 16 zone distances [mm]
 */
LOG_ADD(LOG_FLOAT, s4, &vl53l8cxToFAvg[4])
/**
 * @brief Sensor 5: average of the 16 zone distances [mm]
 */
LOG_ADD(LOG_FLOAT, s5, &vl53l8cxToFAvg[5])
/**
 * @brief Sensor 6: average of the 16 zone distances [mm]
 */
LOG_ADD(LOG_FLOAT, s6, &vl53l8cxToFAvg[6])
/**
 * @brief Sensor 7: average of the 16 zone distances [mm]
 */
LOG_ADD(LOG_FLOAT, s7, &vl53l8cxToFAvg[7])
/**
 * @brief Sensor 8: average of the 16 zone distances [mm]
 */
LOG_ADD(LOG_FLOAT, s8, &vl53l8cxToFAvg[8])
/**
 * @brief Sensor 9: average of the 16 zone distances [mm]
 */
LOG_ADD(LOG_FLOAT, s9, &vl53l8cxToFAvg[9])
/**
 * @brief Sensor 10: average of the 16 zone distances [mm]
 */
LOG_ADD(LOG_FLOAT, s10, &vl53l8cxToFAvg[10])
LOG_GROUP_STOP(vl53l8cx)

/* ===== Logs: per-zone raw distance [mm] and target_status for sensors 0 and 1 ===== */
// LOG_GROUP_START(vl53l8cx_s0)
// LOG_ADD(LOG_INT16, d0,  &vl53l8cxToFDist[0][0])
// LOG_ADD(LOG_INT16, d1,  &vl53l8cxToFDist[0][1])
// LOG_ADD(LOG_INT16, d2,  &vl53l8cxToFDist[0][2])
// LOG_ADD(LOG_INT16, d3,  &vl53l8cxToFDist[0][3])
// LOG_ADD(LOG_INT16, d4,  &vl53l8cxToFDist[0][4])
// LOG_ADD(LOG_INT16, d5,  &vl53l8cxToFDist[0][5])
// LOG_ADD(LOG_INT16, d6,  &vl53l8cxToFDist[0][6])
// LOG_ADD(LOG_INT16, d7,  &vl53l8cxToFDist[0][7])
// LOG_ADD(LOG_INT16, d8,  &vl53l8cxToFDist[0][8])
// LOG_ADD(LOG_INT16, d9,  &vl53l8cxToFDist[0][9])
// LOG_ADD(LOG_INT16, d10, &vl53l8cxToFDist[0][10])
// LOG_ADD(LOG_INT16, d11, &vl53l8cxToFDist[0][11])
// LOG_ADD(LOG_INT16, d12, &vl53l8cxToFDist[0][12])
// LOG_ADD(LOG_INT16, d13, &vl53l8cxToFDist[0][13])
// LOG_ADD(LOG_INT16, d14, &vl53l8cxToFDist[0][14])
// LOG_ADD(LOG_INT16, d15, &vl53l8cxToFDist[0][15])
// LOG_ADD(LOG_UINT8, st0,  &vl53l8cxToFStatus[0][0])
// LOG_ADD(LOG_UINT8, st1,  &vl53l8cxToFStatus[0][1])
// LOG_ADD(LOG_UINT8, st2,  &vl53l8cxToFStatus[0][2])
// LOG_ADD(LOG_UINT8, st3,  &vl53l8cxToFStatus[0][3])
// LOG_ADD(LOG_UINT8, st4,  &vl53l8cxToFStatus[0][4])
// LOG_ADD(LOG_UINT8, st5,  &vl53l8cxToFStatus[0][5])
// LOG_ADD(LOG_UINT8, st6,  &vl53l8cxToFStatus[0][6])
// LOG_ADD(LOG_UINT8, st7,  &vl53l8cxToFStatus[0][7])
// LOG_ADD(LOG_UINT8, st8,  &vl53l8cxToFStatus[0][8])
// LOG_ADD(LOG_UINT8, st9,  &vl53l8cxToFStatus[0][9])
// LOG_ADD(LOG_UINT8, st10, &vl53l8cxToFStatus[0][10])
// LOG_ADD(LOG_UINT8, st11, &vl53l8cxToFStatus[0][11])
// LOG_ADD(LOG_UINT8, st12, &vl53l8cxToFStatus[0][12])
// LOG_ADD(LOG_UINT8, st13, &vl53l8cxToFStatus[0][13])
// LOG_ADD(LOG_UINT8, st14, &vl53l8cxToFStatus[0][14])
// LOG_ADD(LOG_UINT8, st15, &vl53l8cxToFStatus[0][15])
// LOG_GROUP_STOP(vl53l8cx_s0)

// LOG_GROUP_START(vl53l8cx_s1)
// LOG_ADD(LOG_INT16, d0,  &vl53l8cxToFDist[1][0])
// LOG_ADD(LOG_INT16, d1,  &vl53l8cxToFDist[1][1])
// LOG_ADD(LOG_INT16, d2,  &vl53l8cxToFDist[1][2])
// LOG_ADD(LOG_INT16, d3,  &vl53l8cxToFDist[1][3])
// LOG_ADD(LOG_INT16, d4,  &vl53l8cxToFDist[1][4])
// LOG_ADD(LOG_INT16, d5,  &vl53l8cxToFDist[1][5])
// LOG_ADD(LOG_INT16, d6,  &vl53l8cxToFDist[1][6])
// LOG_ADD(LOG_INT16, d7,  &vl53l8cxToFDist[1][7])
// LOG_ADD(LOG_INT16, d8,  &vl53l8cxToFDist[1][8])
// LOG_ADD(LOG_INT16, d9,  &vl53l8cxToFDist[1][9])
// LOG_ADD(LOG_INT16, d10, &vl53l8cxToFDist[1][10])
// LOG_ADD(LOG_INT16, d11, &vl53l8cxToFDist[1][11])
// LOG_ADD(LOG_INT16, d12, &vl53l8cxToFDist[1][12])
// LOG_ADD(LOG_INT16, d13, &vl53l8cxToFDist[1][13])
// LOG_ADD(LOG_INT16, d14, &vl53l8cxToFDist[1][14])
// LOG_ADD(LOG_INT16, d15, &vl53l8cxToFDist[1][15])
// LOG_ADD(LOG_UINT8, st0,  &vl53l8cxToFStatus[1][0])
// LOG_ADD(LOG_UINT8, st1,  &vl53l8cxToFStatus[1][1])
// LOG_ADD(LOG_UINT8, st2,  &vl53l8cxToFStatus[1][2])
// LOG_ADD(LOG_UINT8, st3,  &vl53l8cxToFStatus[1][3])
// LOG_ADD(LOG_UINT8, st4,  &vl53l8cxToFStatus[1][4])
// LOG_ADD(LOG_UINT8, st5,  &vl53l8cxToFStatus[1][5])
// LOG_ADD(LOG_UINT8, st6,  &vl53l8cxToFStatus[1][6])
// LOG_ADD(LOG_UINT8, st7,  &vl53l8cxToFStatus[1][7])
// LOG_ADD(LOG_UINT8, st8,  &vl53l8cxToFStatus[1][8])
// LOG_ADD(LOG_UINT8, st9,  &vl53l8cxToFStatus[1][9])
// LOG_ADD(LOG_UINT8, st10, &vl53l8cxToFStatus[1][10])
// LOG_ADD(LOG_UINT8, st11, &vl53l8cxToFStatus[1][11])
// LOG_ADD(LOG_UINT8, st12, &vl53l8cxToFStatus[1][12])
// LOG_ADD(LOG_UINT8, st13, &vl53l8cxToFStatus[1][13])
// LOG_ADD(LOG_UINT8, st14, &vl53l8cxToFStatus[1][14])
// LOG_ADD(LOG_UINT8, st15, &vl53l8cxToFStatus[1][15])
// LOG_GROUP_STOP(vl53l8cx_s1)

// LOG_GROUP_START(vl53l8cx_s8)
// LOG_ADD(LOG_INT16, d0,  &vl53l8cxToFDist[8][0])
// LOG_ADD(LOG_INT16, d1,  &vl53l8cxToFDist[8][1])
// LOG_ADD(LOG_INT16, d2,  &vl53l8cxToFDist[8][2])
// LOG_ADD(LOG_INT16, d3,  &vl53l8cxToFDist[8][3])
// LOG_ADD(LOG_INT16, d4,  &vl53l8cxToFDist[8][4])
// LOG_ADD(LOG_INT16, d5,  &vl53l8cxToFDist[8][5])
// LOG_ADD(LOG_INT16, d6,  &vl53l8cxToFDist[8][6])
// LOG_ADD(LOG_INT16, d7,  &vl53l8cxToFDist[8][7])
// LOG_ADD(LOG_INT16, d8,  &vl53l8cxToFDist[8][8])
// LOG_ADD(LOG_INT16, d9,  &vl53l8cxToFDist[8][9])
// LOG_ADD(LOG_INT16, d10, &vl53l8cxToFDist[8][10])
// LOG_ADD(LOG_INT16, d11, &vl53l8cxToFDist[8][11])
// LOG_ADD(LOG_INT16, d12, &vl53l8cxToFDist[8][12])
// LOG_ADD(LOG_INT16, d13, &vl53l8cxToFDist[8][13])
// LOG_ADD(LOG_INT16, d14, &vl53l8cxToFDist[8][14])
// LOG_ADD(LOG_INT16, d15, &vl53l8cxToFDist[8][15])
// LOG_ADD(LOG_UINT8, st0,  &vl53l8cxToFStatus[8][0])
// LOG_ADD(LOG_UINT8, st1,  &vl53l8cxToFStatus[8][1])
// LOG_ADD(LOG_UINT8, st2,  &vl53l8cxToFStatus[8][2])
// LOG_ADD(LOG_UINT8, st3,  &vl53l8cxToFStatus[8][3])
// LOG_ADD(LOG_UINT8, st4,  &vl53l8cxToFStatus[8][4])
// LOG_ADD(LOG_UINT8, st5,  &vl53l8cxToFStatus[8][5])
// LOG_ADD(LOG_UINT8, st6,  &vl53l8cxToFStatus[8][6])
// LOG_ADD(LOG_UINT8, st7,  &vl53l8cxToFStatus[8][7])
// LOG_ADD(LOG_UINT8, st8,  &vl53l8cxToFStatus[8][8])
// LOG_ADD(LOG_UINT8, st9,  &vl53l8cxToFStatus[8][9])
// LOG_ADD(LOG_UINT8, st10, &vl53l8cxToFStatus[8][10])
// LOG_ADD(LOG_UINT8, st11, &vl53l8cxToFStatus[8][11])
// LOG_ADD(LOG_UINT8, st12, &vl53l8cxToFStatus[8][12])
// LOG_ADD(LOG_UINT8, st13, &vl53l8cxToFStatus[8][13])
// LOG_ADD(LOG_UINT8, st14, &vl53l8cxToFStatus[8][14])
// LOG_ADD(LOG_UINT8, st15, &vl53l8cxToFStatus[8][15])
// LOG_GROUP_STOP(vl53l8cx_s8)