#define VL53L8CX_BUFFERS_DEFINE
#include "vl53l8cx_buffers.h"

/* Define buffer size symbols so other translation units can query sizes
 * without using sizeof() on an incomplete extern array. The arrays above
 * are fully defined in this TU, so sizeof is valid here. */
const unsigned int VL53L8CX_FIRMWARE_SIZE = sizeof(VL53L8CX_FIRMWARE);
const unsigned int VL53L8CX_DEFAULT_CONFIGURATION_SIZE = sizeof(VL53L8CX_DEFAULT_CONFIGURATION);
const unsigned int VL53L8CX_DEFAULT_XTALK_SIZE = sizeof(VL53L8CX_DEFAULT_XTALK);
const unsigned int VL53L8CX_GET_NVM_CMD_SIZE = sizeof(VL53L8CX_GET_NVM_CMD);
