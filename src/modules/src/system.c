/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * system.c - Top level module implementation
 */
#define DEBUG_MODULE "SYS"

#include <stdbool.h>

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "debug.h"
#include "version.h"
#include "config.h"
#include "param.h"
#include "log.h"
#include "ledseq.h"
#include "pm.h"

#include "system.h"
#include "platform.h"
#include "storage.h"
#include "configblock.h"
#include "worker.h"
#include "freeRTOSdebug.h"
#include "uart_syslink.h"
#include "uart1.h"
#include "uart2.h"
#include "comm.h"
#include "stabilizer.h"
#include "commander.h"
#include "console.h"
#include "usblink.h"
#include "mem.h"
#include "crtp_mem.h"
#include "proximity.h"
#include "watchdog.h"
#include "queuemonitor.h"
#include "buzzer.h"
#include "sound.h"
#include "sysload.h"
#include "estimator_kalman.h"
#include "estimator_ukf.h"
#include "deck.h"
#include "extrx.h"
#include "app.h"
#include "static_mem.h"
#include "peer_localization.h"
#include "usec_time.h"
#include "cfassert.h"
#include "i2cdev.h"
#include "autoconf.h"
#include "vcp_esc_passthrough.h"
#if CONFIG_ENABLE_CPX
  #include "cpxlink.h"
#endif

#include "vl53l8cx_api.h"
#include "../deck/interface/deck_spi.h"
#include "stm32f4xx_spi.h"

#include "crtp_commander_high_level.h"
#include "supervisor.h"

/* Private variable */
static bool selftestPassed;
static uint8_t dumpAssertInfo = 0;
static bool isInit;

static char nrf_version[16];
static uint8_t testLogParam;
static uint8_t doAssert;

STATIC_MEM_TASK_ALLOC(systemTask, SYSTEM_TASK_STACKSIZE);

/* System wide synchronisation */
xSemaphoreHandle canStartMutex;
static StaticSemaphore_t canStartMutexBuffer;
static xSemaphoreHandle vl53l8cxInitDoneSem;
static StaticSemaphore_t vl53l8cxInitDoneSemBuffer;
static xSemaphoreHandle vl53l8cxGetTofDoneSem;
static StaticSemaphore_t vl53l8cxGetTofDoneSemBuffer;

/* Private functions */
static void systemTask(void *arg);
void vl53l8cx_get_tof_task(void *arg);

/* Public functions */
void systemLaunch(void)
{
  STATIC_MEM_TASK_CREATE(systemTask, systemTask, SYSTEM_TASK_NAME, NULL, SYSTEM_TASK_PRI);
}

// This must be the first module to be initialized!
void systemInit(void)
{
  if(isInit)
    return;

  canStartMutex = xSemaphoreCreateMutexStatic(&canStartMutexBuffer);
  xSemaphoreTake(canStartMutex, portMAX_DELAY);

  usblinkInit();
  sysLoadInit();
#if CONFIG_ENABLE_CPX
  cpxlinkInit();
#endif

  /* Initialized here so that DEBUG_PRINT (buffered) can be used early */
  debugInit();
  crtpInit();
  consoleInit();

  // DEBUG_PRINT("----------------------------\n");
  // DEBUG_PRINT("%s is up and running!\n", platformConfigGetDeviceTypeName());

  // if (V_PRODUCTION_RELEASE) {
  //   DEBUG_PRINT("Production release %s\n", V_STAG);
  // } else {
  //   DEBUG_PRINT("Build %s:%s (%s) %s\n", V_SLOCAL_REVISION,
  //               V_SREVISION, V_STAG, (V_MODIFIED)?"MODIFIED":"CLEAN");
  // }
  // DEBUG_PRINT("I am 0x%08X%08X%08X and I have %dKB of flash!\n",
  //             *((int*)(MCU_ID_ADDRESS+8)), *((int*)(MCU_ID_ADDRESS+4)),
  //             *((int*)(MCU_ID_ADDRESS+0)), *((short*)(MCU_FLASH_SIZE_ADDRESS)));

  configblockInit();
  storageInit();
  workerInit();
  adcInit();
  ledseqInit();
  pmInit();
  buzzerInit();
  peerLocalizationInit();

#ifdef CONFIG_APP_ENABLE
  appInit();
#endif

  isInit = true;
}

bool systemTest()
{
  bool pass=isInit;

  pass &= ledseqTest();
  pass &= pmTest();
  pass &= workerTest();
  pass &= buzzerTest();
  return pass;
}

int Ranging_Basic_init(uint16_t DevAddr, VL53L8CX_Configuration* Dev)
{
    uint8_t status = 1;
    uint8_t isAlive = 0;
    Dev->platform.address = DevAddr;

    // (Optional) Check if there is a VL53L8CX sensor connected
    status = vl53l8cx_is_alive(Dev, &isAlive);
    if (isAlive == 0 || status != VL53L8CX_STATUS_OK)
    {
        led_debug(3000, 1,LED_BLUE_L);
        return 0;
    }

    status = vl53l8cx_init(Dev);
    if (status != VL53L8CX_STATUS_OK)
    {
        led_debug(1000, 3, LED_BLUE_L);
        return 0;
    }

    status = vl53l8cx_set_ranging_frequency_hz(Dev, 30);
    if (status != VL53L8CX_STATUS_OK)
    {
        led_debug(3000, 6, LED_BLUE_L);
        return 0;
    }else{
        led_debug(100, (DevAddr+1) * 10, LED_GREEN_L);
        return 1;
    }
}

uint8_t DevAddr[11];
/* Gget_Ranging() の所要時間[us]。vl53l8cx_get_tof_task で計測し SD/ログに出す。 */
uint32_t vl53l8cxRangingUs = 0;
void Gget_Ranging()
{
    // uint8_t status, loop, isAlive, isReady;
    // char i;
    int k;

    for (k = 0; k < vl53l8cx_NUM_SENSORS; k++) DevAddr[k] = 0xFF;
    k = Ser_IT();  // In The platform.cpp

    if (k == 0)
    {
        return;
    }

    if ((k & 0x0020) != 0) DevAddr[10] = 10;
    if ((k & 0x0040) != 0) DevAddr[9] = 9;
    if ((k & 0x0080) != 0) DevAddr[8] = 8;
    if ((k & 0x0100) != 0) DevAddr[7] = 7;
    if ((k & 0x0200) != 0) DevAddr[6] = 6;
    if ((k & 0x0400) != 0) DevAddr[5] = 5;
    if ((k & 0x0800) != 0) DevAddr[4] = 4;
    if ((k & 0x1000) != 0) DevAddr[3] = 3;
    if ((k & 0x2000) != 0) DevAddr[2] = 2;
    if ((k & 0x4000) != 0) DevAddr[1] = 1;
    if ((k & 0x8000) != 0) DevAddr[0] = 0;
    for (k = 0; k < vl53l8cx_NUM_SENSORS; k++)
    {
        if (DevAddr[k] != 0xFF)
        {
            MDev[DevAddr[k]].platform.address = DevAddr[k];
            vl53l8cx_get_ranging_data(&MDev[DevAddr[k]], &Results);

            int32_t tofTotal = 0;
            for (int z = 0; z < 16; z++)
            {
                int16_t dist = Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * z];
                uint8_t st = Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * z];
                vl53l8cxToFDist[DevAddr[k]][z] = dist;
                vl53l8cxToFStatus[DevAddr[k]][z] = st;
                tofTotal += dist;
            }
            vl53l8cxToFAvg[DevAddr[k]] = tofTotal / 16.0f;
            /* このセンサーだけが新しい測距値を得た。ToF オドメトリは
             * このカウンタを見て、更新のあったセンサーだけを使う。 */
            vl53l8cxSensorSeq[DevAddr[k]]++;

            // led_debug(200, (DevAddr[k] + 1) * 5, LED_BLUE_L);
            // for (i = 0; i < 16; i++)
            // {
            //     if (Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i] == 5)
            //     {
            //         if(100 < Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i] && Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i] < 300){
            //         led_debug(200, 5 * (DevAddr[k]+1), LED_GREEN_L);
            //         // DEBUG_PRINT("%d ", Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
            //         }else{
            //         led_debug(200, 5 * (DevAddr[k]+1), LED_GREEN_R);
            //         // DEBUG_PRINT("%d ", Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
            //         }
            //     }
            //     else
            //     {
            //         led_debug(200, 5 * (DevAddr[k]+1), LED_BLUE_L);
            //         // DEBUG_PRINT("er");
            //     }
            //     // DEBUG_PRINT("\n");
            // }
        }
    }

    /* 1スイープ完了。ToF オドメトリタスクがこのカウンタで新フレームを検出する。 */
    vl53l8cxFrameSeq++;
}

void Gget_Ranging_init()
{
  int i = 0;
  uint8_t isInited = 0;
  // この部分は謎
  while(i < vl53l8cx_NUM_SENSORS){
    isInited = Ranging_Basic_init(i, &MDev[i]);
    if(isInited == 1){
      i++;
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
}


void vl53l8cxInitTask(void *param){
  (void)param;
  init_IO();
  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  Gget_Ranging_init();
  spiEndTransaction();
  xSemaphoreGive(vl53l8cxInitDoneSem);
  vTaskDelete(NULL);
}

void vl53l8cx_get_tof_task(void *param){
  (void)param;
  xSemaphoreTake(vl53l8cxInitDoneSem, portMAX_DELAY);
  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  uint8_t status,addr;
  for(addr = 0; addr < vl53l8cx_NUM_SENSORS; addr++){
        status = vl53l8cx_start_ranging(&MDev[addr]);
        if(status){
            ledClearAll();
            led_debug(2000, addr+3, LED_BLUE_L);
        }
    }
  spiEndTransaction();
    xSemaphoreGive(vl53l8cxGetTofDoneSem);
    // 正確に10Hzで回すため、処理時間を吸収する vTaskDelayUntil を使う
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(200);  // 5Hz
    while(1){
        spiBeginTransaction(SPI_BAUDRATE_2MHZ);
        uint64_t rangingStart = usecTimestamp();
        Gget_Ranging();
        vl53l8cxRangingUs = (uint32_t)(usecTimestamp() - rangingStart);
        spiEndTransaction();
        vTaskDelayUntil(&xLastWakeTime, xPeriod);  // 前回起床から100ms周期
    }
}

void flightTask(void *param)
{
  /* Simple scripted flight: wait for system start, takeoff, hover 3s, land */
  systemWaitStart();
  /* short delay to let other subsystems initialize */
  ledClearAll();
  led_debug(1000, 5, LED_BLUE_L);

  /* Arm the system before flight. Brushless platforms (e.g. CF2.1 Brushless)
   * require explicit arming before the motors will spin; brushed platforms
   * also accept the request. Wait until arming succeeds. */
  while (!supervisorIsArmed()) {
    supervisorRequestArming(true);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // /* Takeoff to 0.5 m over 1.0 s */
  // crtpCommanderHighLevelTakeoff(0.5f, 1.0f);

  // /* Hover for 3 seconds */
  // vTaskDelay(pdMS_TO_TICKS(6000));

  // /* Land to ground (0.0 m) over 1.0 s */
  // crtpCommanderHighLevelLand(0.0f, 1.0f);

  vTaskDelete(NULL);
}


/* Private functions implementation */

void systemTask(void *arg)
{
  bool pass = true;

  ledInit();

  vl53l8cxInitDoneSem = xSemaphoreCreateBinaryStatic(&vl53l8cxInitDoneSemBuffer);
  ASSERT(vl53l8cxInitDoneSem);
  vl53l8cxGetTofDoneSem = xSemaphoreCreateBinaryStatic(&vl53l8cxGetTofDoneSemBuffer);
  ASSERT(vl53l8cxGetTofDoneSem);

  if (xTaskCreate(vl53l8cxInitTask, "vl53l8cxInitTask", 512, NULL, 2, NULL) != pdPASS) {
    while(1);
  }
  xTaskCreate(vl53l8cx_get_tof_task, "vl53l8cx_get_tof_task", 1024, NULL, 2, NULL);
  xSemaphoreTake(vl53l8cxGetTofDoneSem, portMAX_DELAY);

  ledSet(CHG_LED, 1);

#ifdef CONFIG_DEBUG_QUEUE_MONITOR
  queueMonitorInit();
#endif

#ifdef CONFIG_DEBUG_PRINT_ON_UART1
  uart1Init(CONFIG_DEBUG_PRINT_ON_UART1_BAUDRATE);
#endif

  usecTimerInit();
  i2cdevInit(I2C3_DEV);
  i2cdevInit(I2C1_DEV);
  passthroughInit();

  //Init the high-levels modules
  systemInit();
  commInit();
  commanderInit();

  StateEstimatorType estimator = StateEstimatorTypeAutoSelect;

  #ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
  estimatorKalmanTaskInit();
  #endif

  #ifdef CONFIG_ESTIMATOR_UKF_ENABLE
  errorEstimatorUkfTaskInit();
  #endif

  // Enabling incoming syslink messages to be added to the queue.
  // This should probably be done later, but deckInit() takes a long time if this is done later.
  uartslkEnableIncoming();

  memInit();
  deckInit();
  estimator = deckGetRequiredEstimator();
  stabilizerInit(estimator);
  if (deckGetRequiredLowInterferenceRadioMode() && platformConfigPhysicalLayoutAntennasAreClose())
  {
    platformSetLowInterferenceRadioMode();
  }
  soundInit();
  crtpMemInit();

#ifdef PROXIMITY_ENABLED
  proximityInit();
#endif

  systemRequestNRFVersion();

  //Test the modules
  DEBUG_PRINT("About to run tests in system.c.\n");
  if (systemTest() == false) {
    pass = false;
    DEBUG_PRINT("system [FAIL]\n");
  }
  if (configblockTest() == false) {
    pass = false;
    DEBUG_PRINT("configblock [FAIL]\n");
  }
  if (storageTest() == false) {
    pass = false;
    DEBUG_PRINT("storage [FAIL]\n");
  }
  if (commTest() == false) {
    pass = false;
    DEBUG_PRINT("comm [FAIL]\n");
  }
  if (commanderTest() == false) {
    pass = false;
    DEBUG_PRINT("commander [FAIL]\n");
  }
  if (stabilizerTest() == false) {
    pass = false;
    DEBUG_PRINT("stabilizer [FAIL]\n");
  }

  #ifdef CONFIG_ESTIMATOR_KALMAN_ENABLE
  if (estimatorKalmanTaskTest() == false) {
    pass = false;
    DEBUG_PRINT("estimatorKalmanTask [FAIL]\n");
  }
  #endif

  #ifdef CONFIG_ESTIMATOR_UKF_ENABLE
  if (errorEstimatorUkfTaskTest() == false) {
    pass = false;
    DEBUG_PRINT("estimatorUKFTask [FAIL]\n");
  }
  #endif
  
  if (deckTest() == false) {
    pass = false;
    DEBUG_PRINT("deck [FAIL]\n");
    led_debug(2000, 2, LED_BLUE_L);
  }
  if (soundTest() == false) {
    pass = false;
    DEBUG_PRINT("sound [FAIL]\n");
    // led_debug(2000, 1, LED_BLUE_L);
  }
  if (memTest() == false) {
    pass = false;
    DEBUG_PRINT("mem [FAIL]\n");
    // led_debug(2000, 1, LED_BLUE_L);
  }
  if (crtpMemTest() == false) {
    pass = false;
    DEBUG_PRINT("CRTP mem [FAIL]\n");
    // led_debug(2000, 1, LED_BLUE_L);
  }
  if (watchdogNormalStartTest() == false) {
    pass = false;
    DEBUG_PRINT("watchdogNormalStart [FAIL]\n");
    // led_debug(2000, 1, LED_BLUE_L);
  }
  if (cfAssertNormalStartTest() == false) {
    pass = false;
    DEBUG_PRINT("cfAssertNormalStart [FAIL]\n");
    // led_debug(2000, 1, LED_BLUE_L);
  }
  if (peerLocalizationTest() == false) {
    pass = false;
    DEBUG_PRINT("peerLocalization [FAIL]\n");
    // led_debug(2000, 1, LED_BLUE_L);
  }
  
  //Start the firmware
  if(pass)
  {
    DEBUG_PRINT("Self test passed!\n");
    selftestPassed = 1;
    systemStart();
    soundSetEffect(SND_STARTUP);
    ledseqRun(&seq_alive);
    ledseqRun(&seq_testPassed);

    // xTaskCreate(flightTask, "FlightTask", 256, NULL, FLOW_TASK_PRI, NULL);
  }
  else
  {
    selftestPassed = 0;
    if (systemTest())
    {
      while(1)
      {
        ledseqRun(&seq_testFailed);
        vTaskDelay(M2T(2000));
        // System can be forced to start by setting the param to 1 from the cfclient
        if (selftestPassed)
        {
	        DEBUG_PRINT("Start forced.\n");
          systemStart();
          break;
        }
      }
    }
    else
    {
      ledInit();
      ledSet(SYS_LED, true);
    }
  }
  DEBUG_PRINT("Free heap: %d bytes\n", xPortGetFreeHeapSize());

  // Notify the nRF51 that we are ready to receive radio packets
  // This is done after systemStart() to ensure all services
  // are ready to process packets, not just queue them.
  // Note: If this is never reached (e.g., self-test failure),
  // the nRF51 will timeout and enable radio anyway for debugging.
  systemSendRadioReady();

  while(1)
    vTaskDelay(portMAX_DELAY);
}


/* Global system variables */
void systemStart()
{
  xSemaphoreGive(canStartMutex);
#ifndef DEBUG
  watchdogInit();
#endif
}

void systemWaitStart(void)
{
  //This permits to guarantee that the system task is initialized before other
  //tasks waits for the start event.
  while(!isInit)
    vTaskDelay(2);

  xSemaphoreTake(canStartMutex, portMAX_DELAY);
  xSemaphoreGive(canStartMutex);
}

void systemRequestShutdown()
{
  SyslinkPacket slp;

  slp.type = SYSLINK_PM_ONOFF_SWITCHOFF;
  slp.length = 0;
  syslinkSendPacket(&slp);
}

void systemRequestNRFVersion()
{
  SyslinkPacket slp;

  slp.type = SYSLINK_SYS_NRF_VERSION;
  slp.length = 0;
  syslinkSendPacket(&slp);
}

void systemSendRadioReady()
{
  SyslinkPacket slp;

  slp.type = SYSLINK_RADIO_READY;
  slp.length = 0;
  syslinkSendPacket(&slp);
}

void systemSyslinkReceive(SyslinkPacket *slp)
{
  if (slp->type == SYSLINK_SYS_NRF_VERSION)
  {
    size_t len = slp->length - 1;

    if (sizeof(nrf_version) - 1 <=  len) {
      len = sizeof(nrf_version) - 1;
    }
    memcpy(&nrf_version, &slp->data[0], len );
    DEBUG_PRINT("NRF51 version: %s\n", nrf_version);
  }
}

void vApplicationIdleHook( void )
{
  static uint32_t tickOfLatestWatchdogReset = M2T(0);

  portTickType tickCount = xTaskGetTickCount();

  if (tickCount - tickOfLatestWatchdogReset > M2T(WATCHDOG_RESET_PERIOD_MS))
  {
    tickOfLatestWatchdogReset = tickCount;
    watchdogReset();
  }

  if (dumpAssertInfo != 0) {
    printAssertSnapshotData();
    dumpAssertInfo = 0;
  }

  // Enter sleep mode. Does not work when debugging chip with SWD.
  // Currently saves about 20mA STM32F405 current consumption (~30%).
#ifndef DEBUG
  { __asm volatile ("wfi"); }
#endif
}

static void doAssertCallback(void) {
  if (doAssert) {
    ASSERT_FAILED();
  }
}

/**
 * This parameter group contain read-only parameters pertaining to the CPU
 * in the Crazyflie.
 *
 * These could be used to identify an unique quad.
 */
PARAM_GROUP_START(cpu)

/**
 * @brief Size in kB of the device flash memory
 */
PARAM_ADD_CORE(PARAM_UINT16 | PARAM_RONLY, flash, MCU_FLASH_SIZE_ADDRESS)

/**
 * @brief Byte `0 - 3` of device unique id
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id0, MCU_ID_ADDRESS+0)

/**
 * @brief Byte `4 - 7` of device unique id
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id1, MCU_ID_ADDRESS+4)

/**
 * @brief Byte `8 - 11` of device unique id
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_RONLY, id2, MCU_ID_ADDRESS+8)

PARAM_GROUP_STOP(cpu)

PARAM_GROUP_START(system)

/**
 * @brief All tests passed when booting
 */
PARAM_ADD_CORE(PARAM_INT8 | PARAM_RONLY, selftestPassed, &selftestPassed)

/**
 * @brief Set to nonzero to trigger dump of assert information to the log.
 */
PARAM_ADD(PARAM_UINT8, assertInfo, &dumpAssertInfo)

/**
 * @brief Test util for log and param. This param sets the value of the sys.testLogParam log variable.
 *
 */
PARAM_ADD(PARAM_UINT8, testLogParam, &testLogParam)

/**
 * @brief Set to non-zero to trigger a failed assert, useful for debugging
 *
 */
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, doAssert, &doAssert, doAssertCallback)


PARAM_GROUP_STOP(system)

/**
 *  System loggable variables to check different system states.
 */
LOG_GROUP_START(sys)
/**
 * @brief Test util for log and param. The value is set through the system.testLogParam parameter
 */
LOG_ADD(LOG_INT8, testLogParam, &testLogParam)

/**
 * @brief Gget_Ranging() の所要時間 [us] (vl53l8cx_get_tof_task内で計測)
 */
// LOG_ADD(LOG_UINT32, rangingUs, &vl53l8cxRangingUs)

LOG_GROUP_STOP(sys)
