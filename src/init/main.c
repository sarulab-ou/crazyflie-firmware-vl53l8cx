/**
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
 * main.c - Containing the main function.
 */

/* Personal configs */
#include "FreeRTOSConfig.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

/* Project includes */
#include "config.h"
#include "platform.h"
#include "system.h"
#include "usec_time.h"

#include "led.h"

/* ST includes */
#include "stm32fxxx.h"

#include "bootloader.h"

#include "vl53l8cx_api.h"
#include "../deck/interface/deck_spi.h"
#include "stm32f4xx_spi.h"
#include "static_mem.h"

VL53L8CX_Configuration Dev;

#define TMP_TASK_STACKSIZE 512
STATIC_MEM_TASK_ALLOC(tmpTask, TMP_TASK_STACKSIZE);

void Ranging_Basic(uint16_t DevAddr)
{
    uint8_t status, loop, isAlive;
    uint8_t isReady;

    Dev.platform.address = DevAddr;

    // (Optional) Check if there is a VL53L8CX sensor connected
    status = vl53l8cx_is_alive(&Dev, &isAlive);
    if (!isAlive || status)
    {
        led_debug(3, 1,LED_BLUE_L);
        return;
    }
    // DEBUG_PRINT("alive\n");
    // (Mandatory) Init VL53L8CX sensor
    status = vl53l8cx_init(&Dev);
    if (status)
    {
        led_debug(3, 3, LED_BLUE_L);
        return;
    }

    // Ranging loop
    status = vl53l8cx_set_ranging_frequency_hz(&Dev, 30);
    if (status)
    {
        // DEBUG_PRINT("set_ranging_frequency_hz failed, status %u\n", status);
        led_debug(3, 6, LED_BLUE_L);
        return;
    }
    status = vl53l8cx_start_ranging(&Dev);
    loop = 0;
    led_debug(2, 2, LED_GREEN_R);
    while (loop < 30000)
    {
        status = vl53l8cx_check_data_ready(&Dev, &isReady);
        if (isReady)
        {
            led_debug(5, 10, LED_GREEN_L);
            // vl53l8cx_get_ranging_data(&Dev, &Results);
            // DEBUG_PRINT("Print data no : %3u\n", Dev.streamcount);
            // for (i = 0; i < 16; i++)
            // {
            //     DEBUG_PRINT("Zone : %3d, Status : %3u, Distance : %4d mm\n", i,
            //                 Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i],
            //                 Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
            // }
            // DEBUG_PRINT("\n");
            loop++;
        }
        // DEBUG_PRINT("l\n");
        VL53L8CX_WaitMs(&(Dev.platform), 50);
    }
}

void tmpTask(void *arg){
  init_IO();
  spiBeginTransaction(SPI_BAUDRATE_2MHZ);
  led_debug(3, 10,LED_BLUE_L);
  Ranging_Basic(0);
  while(1);
}

int main() 
{
  check_enter_bootloader();

  //Initialize the platform.
  int err = platformInit();
  if (err != 0) {
    // The firmware is running on the wrong hardware. Halt
    while(1);
  }
  ledInit();

  STATIC_MEM_TASK_CREATE(tmpTask, tmpTask, "tmp", NULL, 2);

  //Start the FreeRTOS scheduler
  vTaskStartScheduler();

  // //Should never reach this point!
  // while(1);

  return 0;
}

