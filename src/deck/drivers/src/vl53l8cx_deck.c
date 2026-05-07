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

/* ===== Provide your REAL CS pins here ===== */
const deckPin_t g_vl53l8cx_cs[vl53l8cx_NUM_SENSORS] = {
    (deckPin_t){0},
};

/* ===== Driver state (ultra-low RAM) ===== */
// static volatile uint8_t g_running = 0;
// static uint8_t g_rate_hz = vl53l8cx_DEFAULT_RATE_HZ;
// static uint32_t g_tick = 0;


static TaskHandle_t g_task = NULL;
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


/* ===== CS helpers ===== */
static inline void cs_low(int i) { vl53l8cx_GPIO_WRITE(g_vl53l8cx_cs[i], LOW); }
static inline void cs_high(int i) { vl53l8cx_GPIO_WRITE(g_vl53l8cx_cs[i], HIGH); }

void tmp()
{   
    uint32_t total = 0;
    while(1){
        total = 0;
        for(int i = 0; i<16;i++){
            total += Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i];
        }
        // led_debug(1000, (Results.platform.address + 1) * 5, LED_GREEN_L);
        if (100 < (int)(total / 16) && (int)(total / 16) < 300)
        {
            if(Results.platform.address == 0){
                led_debug(200, 5, LED_GREEN_L);
            }else if(Results.platform.address == 1){
                led_debug(200, 5, LED_GREEN_R);
            }
        }else if(total == 0){
            led_debug(1000, 1, LED_BLUE_L);
        }else{
            led_debug(200, (Results.platform.address + 1)*5, LED_BLUE_L);
        }

        // if(Results.target_status[0] != 5 && Results.target_status[0] != 9){
        //     led_debug(200, Results.target_status[0], LED_GREEN_L);
        // }
        // DEBUG_PRINT("%lu\n", total);
        // DEBUG_PRINT("Add%d: %lu\n", Results.platform.address, (int32_t)(total/16));
        // vTaskDelay(pdMS_TO_TICKS(500));
    }
    // uint16_t devAddr= 2;
    // uint8_t loop, isReady;
    // uint8_t addr, status;
    // uint8_t i,j,k;
    // spiBeginTransaction(SPI_BAUDRATE_2MHZ);

    // while(1){
    //     loop = 0;
    //     // led_debug(1000, 10, LED_BLUE_L);
    //     while (loop < 30000)
    //     {
    //         status = vl53l8cx_check_data_ready(&MDev[devAddr], &isReady);
    //         DEBUG_PRINT("Addr%d is %d\n", devAddr, isReady);
    //         if (isReady)
    //         {
    //             vl53l8cx_get_ranging_data(&MDev[devAddr], &Results);
    //             DEBUG_PRINT("dev %3u\n", devAddr);

    //             for(i = 4; 0<i; i--){
    //               for(j = 4; 0<j; j--){
    //                 DEBUG_PRINT("%4d ", Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * (i *j)-1]);
    //               }
    //               DEBUG_PRINT("\n");
    //             }
    //             DEBUG_PRINT("\n");
    //             // for (i = 0; i < 16; i++)
    //             // {
    //             //     DEBUG_PRINT("Zone: %3d, Status: %3u, Dist: %4d mm\n", i,
    //             //                 Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i],
    //             //                 Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
    //             // }
    //             // DEBUG_PRINT("\n");
    //             loop++;
    //             VL53L8CX_WaitMs(&(MDev[devAddr].platform), 500);
    //             break;
                
    //         }
    //         // DEBUG_PRINT("l\n");
    //         VL53L8CX_WaitMs(&(MDev[devAddr].platform), 100);
    //     }
    //     devAddr = (devAddr + 1) % vl53l8cx_NUM_SENSORS;
    // }
    // spiEndTransaction();
}

static void vl53l8cxTask(void* arg)
{
    (void)arg;
    systemWaitStart();
    // int tmp_sensor = 0;
    // for (;;)
    // {
    //     // DEBUG_PRINT("w\n");
    //     vTaskDelay(pdMS_TO_TICKS(100));
    //     // DEBUG_PRINT("w\n");
    //     if (callbacked)
    //     {
    //         // DEBUG_PRINT("b\n");
    //         break;
    //     }
    // }
    ledClearAll();
    // Gget_Ranging();
    tmp();

    // int n = 0;
    // uint8_t status;
    // init_IO();
    // spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    // Ranging_Basic(0);
    // このループは、11個のセンサーがすべて初期化できるまで繰り返す
    // while (1)
    // {
    //     if (Init_Sensor(tmp_sensor, 30))
    //     {
    //         // DEBUG_PRINT("Init_Sensor %d end!!!!!!!!!!!!\n\n", n);
    //         n++;
    //         if (n >= vl53l8cx_NUM_SENSORS)
    //         {
    //             break;
    //         }
    //     }
    //     // vTaskDelay_for_spi_pause(pdMS_TO_TICKS(500));
    //     VL53L8CX_WaitMs_spi_pause(&MDev[tmp_sensor].platform, 500);
    // }

    // led_debug(2);

    // uint8_t isReady = 0;
    // int loop = 0;
    // while (loop < 10)
    // {
    //     status = vl53l8cx_check_data_ready(&MDev[tmp_sensor], &isReady);
    //     if (isReady)
    //     {
    //         vl53l8cx_get_ranging_data(&MDev[tmp_sensor], &Results);
    //         DEBUG_PRINT("Print data no : %3u\n", MDev[tmp_sensor].streamcount);
    //         for (int i = 0; i < 16; i++)
    //         {
    //             DEBUG_PRINT("Zone : %3d, Status : %3u, Distance : %4d mm\n", i,
    //                         Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i],
    //                         Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
    //         }
    //         DEBUG_PRINT("\n");
    //         loop++;
    //     }
    //     VL53L8CX_WaitMs_spi_pause(&(MDev[tmp_sensor].platform), 5);
    // }
    // spiEndTransaction();
    vTaskDelete(NULL);
}

// static void onEnableUpdated()
// {
//     callbacked = 1;
// }

/* ===== Robust blob address snoop (FLASH vs RAM) ===== */
static void printBlobAddresses(void)
{
    if (g_task == NULL)
    // if (g_running && g_task == NULL)
    {
        BaseType_t rc = xTaskCreate(vl53l8cxTask, "vl53l8cx", 512, NULL, tskIDLE_PRIORITY + 2, &g_task);
        if (rc == pdPASS)
        {
            // DEBUG_PRINT("OK\n");
        }
        else
        {
            // DEBUG_PRINT("FAIL\n");
        }
    }
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
    printBlobAddresses();
}

// bool vl53l8cxIsRunning(void) { return g_running != 0; }
// uint16_t vl53l8cxGetLastMm(int index) { if (index < 0 || index >= vl53l8cx_NUM_SENSORS) return 0; return
// g_ranges_mm[index];
// }

static const DeckDriver bcVL53L8CX11 = {.name = "bcVL53L8CX11", .init = vl53l8cxInit};
DECK_DRIVER(bcVL53L8CX11);