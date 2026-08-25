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
#include "tof_odometry.h"
#include "tof_wall_angle.h"
#include "vl53l8cx_api.h"
/* Deck SPI and GPIO APIs */
#include "deck_constants.h"
#include "deck_digital.h"
#include "deck_spi.h"
#include "stm32f4xx_spi.h" /* For SPI_BaudRatePrescaler_* definitions */

#ifndef vl53l8cx_USE_CCM
#define vl53l8cx_USE_CCM 1
#endif
#ifndef vl53l8cx_DEFAULT_RATE_HZ
#define vl53l8cx_DEFAULT_RATE_HZ 10
#endif

/* Extern-only declarations of the ULD blobs. Must be defined exactly once elsewhere. */
#include "platform_vl53l8cx.h"
#include "vl53l8cx_buffers.h"

static TaskHandle_t g_task = NULL;
/* Per-sensor average of the 16 zone distances [mm]. Updated in Gget_Ranging() (system.c). */
float vl53l8cxToFAvg[vl53l8cx_NUM_SENSORS] = {0};
/* Per-sensor raw distance [mm] and target_status of each of the 16 zones. Updated in Gget_Ranging() (system.c). */
int16_t vl53l8cxToFDist[vl53l8cx_NUM_SENSORS][16] = {{0}};
uint8_t vl53l8cxToFStatus[vl53l8cx_NUM_SENSORS][16] = {{0}};
/* Incremented once per completed ranging sweep in Gget_Ranging() (system.c).
 * The odometry task uses it to process each frame exactly once. */
volatile uint32_t vl53l8cxFrameSeq = 0;
/* Incremented each time THIS sensor actually delivers new ranging data.
 * A sweep only reads the sensors whose interrupt fired, so without this the
 * odometry would silently re-use the previous sweep's distances. */
volatile uint32_t vl53l8cxSensorSeq[vl53l8cx_NUM_SENSORS] = {0};

/* ONE shared configuration + ONE shared results */
// NO_DMA_CCM_SAFE_ZERO_INIT static VL53L8CX_Configuration g_dev;
#ifdef vl53l8cx_USE_CCM
__attribute__((section(".ccmram")))
#endif
uint8_t ReStart[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t NumRdy[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};



static void vl53l8cxTask(void* arg)
{
    (void)arg;
    systemWaitStart();

    /* Convert the ToF point clouds into translational and rotational motion
     * estimates. Runs once per ranging sweep (Gget_Ranging(), 5 Hz) rather than
     * on its own timer, so no frame is processed twice or skipped. */
    tofOdometryInit();
#if TOF_WALL_ANGLE_ENABLED
    /* 壁に対する機体のヨーずれを ToF だけで推定する (ジャイロ不使用)。
     * オドメトリとは独立で、結果は log グループ "tofwall" と
     * tofWallAngleGetBodyDeg() から参照できる。 */
    tofWallAngleInit();
#endif
    uint32_t lastSeq = vl53l8cxFrameSeq;

    while (1)
    {
        uint32_t seq = vl53l8cxFrameSeq;
        if (seq != lastSeq)
        {
            lastSeq = seq;
            tofOdometryUpdate();
#if TOF_WALL_ANGLE_ENABLED
            tofWallAngleUpdate();
#endif
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // poll at 20Hz, ranging arrives at 5Hz
    }
    vTaskDelete(NULL);
}

/* ===== Deck glue ===== */
static void vl53l8cxInit(DeckInfo* info)
{
    (void)info;
    /* 768 words: PCA/RANSAC/Kabsch の再帰しない呼び出し鎖で ~700B 使う */
    xTaskCreate(vl53l8cxTask, "vl53l8cx", 768, NULL, tskIDLE_PRIORITY + 2, &g_task);
}

static const DeckDriver bcVL53L8CX11 = {.name = "bcVL53L8CX11", .init = vl53l8cxInit};
DECK_DRIVER(bcVL53L8CX11);

/* ===== Logs: per-sensor averaged ToF distance [mm] ===== */
LOG_GROUP_START(vl53l8cx)
LOG_ADD(LOG_FLOAT, s0, &vl53l8cxToFAvg[0])
LOG_ADD(LOG_FLOAT, s1, &vl53l8cxToFAvg[1])
LOG_ADD(LOG_FLOAT, s2, &vl53l8cxToFAvg[2])
LOG_ADD(LOG_FLOAT, s3, &vl53l8cxToFAvg[3])
LOG_ADD(LOG_FLOAT, s4, &vl53l8cxToFAvg[4])
LOG_ADD(LOG_FLOAT, s5, &vl53l8cxToFAvg[5])
LOG_ADD(LOG_FLOAT, s6, &vl53l8cxToFAvg[6])
LOG_ADD(LOG_FLOAT, s7, &vl53l8cxToFAvg[7])
LOG_ADD(LOG_FLOAT, s8, &vl53l8cxToFAvg[8])
LOG_ADD(LOG_FLOAT, s9, &vl53l8cxToFAvg[9])
LOG_ADD(LOG_FLOAT, s10, &vl53l8cxToFAvg[10])
LOG_GROUP_STOP(vl53l8cx)

/* ===== Logs: per-zone raw distance [mm] and target_status for sensors 0 and 1 ===== */
LOG_GROUP_START(vl53l8cx_s0)
LOG_ADD(LOG_INT16, d0,  &vl53l8cxToFDist[0][0])
LOG_ADD(LOG_INT16, d1,  &vl53l8cxToFDist[0][1])
LOG_ADD(LOG_INT16, d2,  &vl53l8cxToFDist[0][2])
LOG_ADD(LOG_INT16, d3,  &vl53l8cxToFDist[0][3])
LOG_ADD(LOG_INT16, d4,  &vl53l8cxToFDist[0][4])
LOG_ADD(LOG_INT16, d5,  &vl53l8cxToFDist[0][5])
LOG_ADD(LOG_INT16, d6,  &vl53l8cxToFDist[0][6])
LOG_ADD(LOG_INT16, d7,  &vl53l8cxToFDist[0][7])
LOG_ADD(LOG_INT16, d8,  &vl53l8cxToFDist[0][8])
LOG_ADD(LOG_INT16, d9,  &vl53l8cxToFDist[0][9])
LOG_ADD(LOG_INT16, d10, &vl53l8cxToFDist[0][10])
LOG_ADD(LOG_INT16, d11, &vl53l8cxToFDist[0][11])
LOG_ADD(LOG_INT16, d12, &vl53l8cxToFDist[0][12])
LOG_ADD(LOG_INT16, d13, &vl53l8cxToFDist[0][13])
LOG_ADD(LOG_INT16, d14, &vl53l8cxToFDist[0][14])
LOG_ADD(LOG_INT16, d15, &vl53l8cxToFDist[0][15])
LOG_ADD(LOG_UINT8, st0,  &vl53l8cxToFStatus[0][0])
LOG_ADD(LOG_UINT8, st1,  &vl53l8cxToFStatus[0][1])
LOG_ADD(LOG_UINT8, st2,  &vl53l8cxToFStatus[0][2])
LOG_ADD(LOG_UINT8, st3,  &vl53l8cxToFStatus[0][3])
LOG_ADD(LOG_UINT8, st4,  &vl53l8cxToFStatus[0][4])
LOG_ADD(LOG_UINT8, st5,  &vl53l8cxToFStatus[0][5])
LOG_ADD(LOG_UINT8, st6,  &vl53l8cxToFStatus[0][6])
LOG_ADD(LOG_UINT8, st7,  &vl53l8cxToFStatus[0][7])
LOG_ADD(LOG_UINT8, st8,  &vl53l8cxToFStatus[0][8])
LOG_ADD(LOG_UINT8, st9,  &vl53l8cxToFStatus[0][9])
LOG_ADD(LOG_UINT8, st10, &vl53l8cxToFStatus[0][10])
LOG_ADD(LOG_UINT8, st11, &vl53l8cxToFStatus[0][11])
LOG_ADD(LOG_UINT8, st12, &vl53l8cxToFStatus[0][12])
LOG_ADD(LOG_UINT8, st13, &vl53l8cxToFStatus[0][13])
LOG_ADD(LOG_UINT8, st14, &vl53l8cxToFStatus[0][14])
LOG_ADD(LOG_UINT8, st15, &vl53l8cxToFStatus[0][15])
LOG_GROUP_STOP(vl53l8cx_s0)

LOG_GROUP_START(vl53l8cx_s1)
LOG_ADD(LOG_INT16, d0,  &vl53l8cxToFDist[1][0])
LOG_ADD(LOG_INT16, d1,  &vl53l8cxToFDist[1][1])
LOG_ADD(LOG_INT16, d2,  &vl53l8cxToFDist[1][2])
LOG_ADD(LOG_INT16, d3,  &vl53l8cxToFDist[1][3])
LOG_ADD(LOG_INT16, d4,  &vl53l8cxToFDist[1][4])
LOG_ADD(LOG_INT16, d5,  &vl53l8cxToFDist[1][5])
LOG_ADD(LOG_INT16, d6,  &vl53l8cxToFDist[1][6])
LOG_ADD(LOG_INT16, d7,  &vl53l8cxToFDist[1][7])
LOG_ADD(LOG_INT16, d8,  &vl53l8cxToFDist[1][8])
LOG_ADD(LOG_INT16, d9,  &vl53l8cxToFDist[1][9])
LOG_ADD(LOG_INT16, d10, &vl53l8cxToFDist[1][10])
LOG_ADD(LOG_INT16, d11, &vl53l8cxToFDist[1][11])
LOG_ADD(LOG_INT16, d12, &vl53l8cxToFDist[1][12])
LOG_ADD(LOG_INT16, d13, &vl53l8cxToFDist[1][13])
LOG_ADD(LOG_INT16, d14, &vl53l8cxToFDist[1][14])
LOG_ADD(LOG_INT16, d15, &vl53l8cxToFDist[1][15])
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
LOG_GROUP_STOP(vl53l8cx_s1)

LOG_GROUP_START(vl53l8cx_s2)
LOG_ADD(LOG_INT16, d0,  &vl53l8cxToFDist[2][0])
LOG_ADD(LOG_INT16, d1,  &vl53l8cxToFDist[2][1])
LOG_ADD(LOG_INT16, d2,  &vl53l8cxToFDist[2][2])
LOG_ADD(LOG_INT16, d3,  &vl53l8cxToFDist[2][3])
LOG_ADD(LOG_INT16, d4,  &vl53l8cxToFDist[2][4])
LOG_ADD(LOG_INT16, d5,  &vl53l8cxToFDist[2][5])
LOG_ADD(LOG_INT16, d6,  &vl53l8cxToFDist[2][6])
LOG_ADD(LOG_INT16, d7,  &vl53l8cxToFDist[2][7])
LOG_ADD(LOG_INT16, d8,  &vl53l8cxToFDist[2][8])
LOG_ADD(LOG_INT16, d9,  &vl53l8cxToFDist[2][9])
LOG_ADD(LOG_INT16, d10, &vl53l8cxToFDist[2][10])
LOG_ADD(LOG_INT16, d11, &vl53l8cxToFDist[2][11])
LOG_ADD(LOG_INT16, d12, &vl53l8cxToFDist[2][12])
LOG_ADD(LOG_INT16, d13, &vl53l8cxToFDist[2][13])
LOG_ADD(LOG_INT16, d14, &vl53l8cxToFDist[2][14])
LOG_ADD(LOG_INT16, d15, &vl53l8cxToFDist[2][15])
// LOG_ADD(LOG_UINT8, st0,  &vl53l8cxToFStatus[2][0])
// LOG_ADD(LOG_UINT8, st1,  &vl53l8cxToFStatus[2][1])
// LOG_ADD(LOG_UINT8, st2,  &vl53l8cxToFStatus[2][2])
// LOG_ADD(LOG_UINT8, st3,  &vl53l8cxToFStatus[2][3])
// LOG_ADD(LOG_UINT8, st4,  &vl53l8cxToFStatus[2][4])
// LOG_ADD(LOG_UINT8, st5,  &vl53l8cxToFStatus[2][5])
// LOG_ADD(LOG_UINT8, st6,  &vl53l8cxToFStatus[2][6])
// LOG_ADD(LOG_UINT8, st7,  &vl53l8cxToFStatus[2][7])
// LOG_ADD(LOG_UINT8, st8,  &vl53l8cxToFStatus[2][8])
// LOG_ADD(LOG_UINT8, st9,  &vl53l8cxToFStatus[2][9])
// LOG_ADD(LOG_UINT8, st10, &vl53l8cxToFStatus[2][10])
// LOG_ADD(LOG_UINT8, st11, &vl53l8cxToFStatus[2][11])
// LOG_ADD(LOG_UINT8, st12, &vl53l8cxToFStatus[2][12])
// LOG_ADD(LOG_UINT8, st13, &vl53l8cxToFStatus[2][13])
// LOG_ADD(LOG_UINT8, st14, &vl53l8cxToFStatus[2][14])
// LOG_ADD(LOG_UINT8, st15, &vl53l8cxToFStatus[2][15])
LOG_GROUP_STOP(vl53l8cx_s2)

LOG_GROUP_START(vl53l8cx_s5)
LOG_ADD(LOG_INT16, d0,  &vl53l8cxToFDist[5][0])
LOG_ADD(LOG_INT16, d1,  &vl53l8cxToFDist[5][1])
LOG_ADD(LOG_INT16, d2,  &vl53l8cxToFDist[5][2])
LOG_ADD(LOG_INT16, d3,  &vl53l8cxToFDist[5][3])
LOG_ADD(LOG_INT16, d4,  &vl53l8cxToFDist[5][4])
LOG_ADD(LOG_INT16, d5,  &vl53l8cxToFDist[5][5])
LOG_ADD(LOG_INT16, d6,  &vl53l8cxToFDist[5][6])
LOG_ADD(LOG_INT16, d7,  &vl53l8cxToFDist[5][7])
LOG_ADD(LOG_INT16, d8,  &vl53l8cxToFDist[5][8])
LOG_ADD(LOG_INT16, d9,  &vl53l8cxToFDist[5][9])
LOG_ADD(LOG_INT16, d10, &vl53l8cxToFDist[5][10])
LOG_ADD(LOG_INT16, d11, &vl53l8cxToFDist[5][11])
LOG_ADD(LOG_INT16, d12, &vl53l8cxToFDist[5][12])
LOG_ADD(LOG_INT16, d13, &vl53l8cxToFDist[5][13])
LOG_ADD(LOG_INT16, d14, &vl53l8cxToFDist[5][14])
LOG_ADD(LOG_INT16, d15, &vl53l8cxToFDist[5][15])
// LOG_ADD(LOG_UINT8, st0,  &vl53l8cxToFStatus[5][0])
// LOG_ADD(LOG_UINT8, st1,  &vl53l8cxToFStatus[5][1])
// LOG_ADD(LOG_UINT8, st2,  &vl53l8cxToFStatus[5][2])
// LOG_ADD(LOG_UINT8, st3,  &vl53l8cxToFStatus[5][3])
// LOG_ADD(LOG_UINT8, st4,  &vl53l8cxToFStatus[5][4])
// LOG_ADD(LOG_UINT8, st5,  &vl53l8cxToFStatus[5][5])
// LOG_ADD(LOG_UINT8, st6,  &vl53l8cxToFStatus[5][6])
// LOG_ADD(LOG_UINT8, st7,  &vl53l8cxToFStatus[5][7])
// LOG_ADD(LOG_UINT8, st8,  &vl53l8cxToFStatus[5][8])
// LOG_ADD(LOG_UINT8, st9,  &vl53l8cxToFStatus[5][9])
// LOG_ADD(LOG_UINT8, st10, &vl53l8cxToFStatus[5][10])
// LOG_ADD(LOG_UINT8, st11, &vl53l8cxToFStatus[5][11])
// LOG_ADD(LOG_UINT8, st12, &vl53l8cxToFStatus[5][12])
// LOG_ADD(LOG_UINT8, st13, &vl53l8cxToFStatus[5][13])
// LOG_ADD(LOG_UINT8, st14, &vl53l8cxToFStatus[5][14])
// LOG_ADD(LOG_UINT8, st15, &vl53l8cxToFStatus[5][15])
LOG_GROUP_STOP(vl53l8cx_s5)

LOG_GROUP_START(vl53l8cx_s6)
LOG_ADD(LOG_INT16, d0,  &vl53l8cxToFDist[6][0])
LOG_ADD(LOG_INT16, d1,  &vl53l8cxToFDist[6][1])
LOG_ADD(LOG_INT16, d2,  &vl53l8cxToFDist[6][2])
LOG_ADD(LOG_INT16, d3,  &vl53l8cxToFDist[6][3])
LOG_ADD(LOG_INT16, d4,  &vl53l8cxToFDist[6][4])
LOG_ADD(LOG_INT16, d5,  &vl53l8cxToFDist[6][5])
LOG_ADD(LOG_INT16, d6,  &vl53l8cxToFDist[6][6])
LOG_ADD(LOG_INT16, d7,  &vl53l8cxToFDist[6][7])
LOG_ADD(LOG_INT16, d8,  &vl53l8cxToFDist[6][8])
LOG_ADD(LOG_INT16, d9,  &vl53l8cxToFDist[6][9])
LOG_ADD(LOG_INT16, d10, &vl53l8cxToFDist[6][10])
LOG_ADD(LOG_INT16, d11, &vl53l8cxToFDist[6][11])
LOG_ADD(LOG_INT16, d12, &vl53l8cxToFDist[6][12])
LOG_ADD(LOG_INT16, d13, &vl53l8cxToFDist[6][13])
LOG_ADD(LOG_INT16, d14, &vl53l8cxToFDist[6][14])
LOG_ADD(LOG_INT16, d15, &vl53l8cxToFDist[6][15])
// LOG_ADD(LOG_UINT8, st0,  &vl53l8cxToFStatus[6][0])
// LOG_ADD(LOG_UINT8, st1,  &vl53l8cxToFStatus[6][1])
// LOG_ADD(LOG_UINT8, st2,  &vl53l8cxToFStatus[6][2])
// LOG_ADD(LOG_UINT8, st3,  &vl53l8cxToFStatus[6][3])
// LOG_ADD(LOG_UINT8, st4,  &vl53l8cxToFStatus[6][4])
// LOG_ADD(LOG_UINT8, st5,  &vl53l8cxToFStatus[6][5])
// LOG_ADD(LOG_UINT8, st6,  &vl53l8cxToFStatus[6][6])
// LOG_ADD(LOG_UINT8, st7,  &vl53l8cxToFStatus[6][7])
// LOG_ADD(LOG_UINT8, st8,  &vl53l8cxToFStatus[6][8])
// LOG_ADD(LOG_UINT8, st9,  &vl53l8cxToFStatus[6][9])
// LOG_ADD(LOG_UINT8, st10, &vl53l8cxToFStatus[6][10])
// LOG_ADD(LOG_UINT8, st11, &vl53l8cxToFStatus[6][11])
// LOG_ADD(LOG_UINT8, st12, &vl53l8cxToFStatus[6][12])
// LOG_ADD(LOG_UINT8, st13, &vl53l8cxToFStatus[6][13])
// LOG_ADD(LOG_UINT8, st14, &vl53l8cxToFStatus[6][14])
// LOG_ADD(LOG_UINT8, st15, &vl53l8cxToFStatus[6][15])
LOG_GROUP_STOP(vl53l8cx_s6)

LOG_GROUP_START(vl53l8cx_s7)
LOG_ADD(LOG_INT16, d0,  &vl53l8cxToFDist[7][0])
LOG_ADD(LOG_INT16, d1,  &vl53l8cxToFDist[7][1])
LOG_ADD(LOG_INT16, d2,  &vl53l8cxToFDist[7][2])
LOG_ADD(LOG_INT16, d3,  &vl53l8cxToFDist[7][3])
LOG_ADD(LOG_INT16, d4,  &vl53l8cxToFDist[7][4])
LOG_ADD(LOG_INT16, d5,  &vl53l8cxToFDist[7][5])
LOG_ADD(LOG_INT16, d6,  &vl53l8cxToFDist[7][6])
LOG_ADD(LOG_INT16, d7,  &vl53l8cxToFDist[7][7])
LOG_ADD(LOG_INT16, d8,  &vl53l8cxToFDist[7][8])
LOG_ADD(LOG_INT16, d9,  &vl53l8cxToFDist[7][9])
LOG_ADD(LOG_INT16, d10, &vl53l8cxToFDist[7][10])
LOG_ADD(LOG_INT16, d11, &vl53l8cxToFDist[7][11])
LOG_ADD(LOG_INT16, d12, &vl53l8cxToFDist[7][12])
LOG_ADD(LOG_INT16, d13, &vl53l8cxToFDist[7][13])
LOG_ADD(LOG_INT16, d14, &vl53l8cxToFDist[7][14])
LOG_ADD(LOG_INT16, d15, &vl53l8cxToFDist[7][15])
// LOG_ADD(LOG_UINT8, st0,  &vl53l8cxToFStatus[7][0])
// LOG_ADD(LOG_UINT8, st1,  &vl53l8cxToFStatus[7][1])
// LOG_ADD(LOG_UINT8, st2,  &vl53l8cxToFStatus[7][2])
// LOG_ADD(LOG_UINT8, st3,  &vl53l8cxToFStatus[7][3])
// LOG_ADD(LOG_UINT8, st4,  &vl53l8cxToFStatus[7][4])
// LOG_ADD(LOG_UINT8, st5,  &vl53l8cxToFStatus[7][5])
// LOG_ADD(LOG_UINT8, st6,  &vl53l8cxToFStatus[7][6])
// LOG_ADD(LOG_UINT8, st7,  &vl53l8cxToFStatus[7][7])
// LOG_ADD(LOG_UINT8, st8,  &vl53l8cxToFStatus[7][8])
// LOG_ADD(LOG_UINT8, st9,  &vl53l8cxToFStatus[7][9])
// LOG_ADD(LOG_UINT8, st10, &vl53l8cxToFStatus[7][10])
// LOG_ADD(LOG_UINT8, st11, &vl53l8cxToFStatus[7][11])
// LOG_ADD(LOG_UINT8, st12, &vl53l8cxToFStatus[7][12])
// LOG_ADD(LOG_UINT8, st13, &vl53l8cxToFStatus[7][13])
// LOG_ADD(LOG_UINT8, st14, &vl53l8cxToFStatus[7][14])
// LOG_ADD(LOG_UINT8, st15, &vl53l8cxToFStatus[7][15])
LOG_GROUP_STOP(vl53l8cx_s7)