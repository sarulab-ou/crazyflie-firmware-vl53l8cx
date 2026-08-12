/**
 * taskload.c - Per-task CPU execution-time logger for microSD logging.
 *
 * FreeRTOS is built here with configGENERATE_RUN_TIME_STATS=1 and
 * portGET_RUN_TIME_COUNTER_VALUE()=usecTimestamp() (see config.h), so
 * every task carries a cumulative run-time counter in *microseconds* of
 * CPU time. This module snapshots all tasks every `periodMs` and, for
 * each task of interest, publishes the CPU time it consumed during that
 * interval (microseconds) as a LOG variable named after the task.
 *
 * Interpretation:
 *   - taskload.<TASK>  = microseconds of CPU time that task used in the
 *                        last interval. With the default 1000 ms period
 *                        this is simply "CPU microseconds per second".
 *                        Divide by (periodMs*1000) for a 0..1 duty ratio.
 *   - taskload.cpuLoad = overall CPU utilisation in % (100*(1 - idle)).
 *   - taskload.total   = sum of all tasks' CPU time in the interval (us).
 *   - taskload.period  = actual measured interval length (us), for exact
 *                        normalisation.
 *
 * This covers ALL continuously-looping tasks with the three decks fitted:
 * the flight stack (STABILIZER runs the EKF-fed controller + motor power
 * distribution, KALMAN runs the estimator, SENSORS reads the IMU), the
 * deck drivers (FLOW + ZRANGER2 for the flow deck, VL53L8CX for the
 * multizone ToF deck, USDLOG + USDWRITE for the microSD deck) and the
 * system/comms tasks. Note the counter includes time spent in interrupts
 * charged to the running task, same caveat as the built-in task dump.
 *
 * To record to the SD card, add the desired "taskload.<name>" entries to
 * the card's config.txt (see tools/usdlog/config.txt for the list added).
 */

#define DEBUG_MODULE "TASKLOAD"

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "debug.h"
#include "cfassert.h"
#include "log.h"
#include "param.h"
#include "static_mem.h"
#include "usec_time.h"

#include "taskload.h"

/* Every task we publish a dedicated LOG variable for. `logName` is the
 * name that appears as "taskload.<logName>" (kept as a valid identifier),
 * `taskName` is the exact FreeRTOS task name to match (config.h). */
typedef struct {
  const char* logName;
  const char* taskName;
  uint32_t prevRunTime;   /* previous cumulative counter (us) */
  uint32_t deltaUs;       /* CPU time used in the last interval (us) */
  uint16_t stackFreeB;    /* minimum-ever free stack of this task (bytes) */
  bool seen;              /* task was present in the last snapshot */
} taskEntry_t;

/* NOTE: order here defines the LOG variable order; add/remove freely.
 * Names not present at runtime simply stay at 0. */
static taskEntry_t entries[] = {
  /* --- Flight stack --- */
  { "STABILIZER", STABILIZER_TASK_NAME,    0, 0, 0, false },  /* control + motors */
  { "KALMAN",     KALMAN_TASK_NAME,        0, 0, 0, false },  /* EKF estimator */
  { "SENSORS",    SENSORS_TASK_NAME,       0, 0, 0, false },  /* IMU acquisition */
  { "PWRMGNT",    PM_TASK_NAME,            0, 0, 0, false },
  { "CMDHL",      CMD_HIGH_LEVEL_TASK_NAME,  0, 0, 0, false },
  { "APP",        APP_TASK_NAME,           0, 0, 0, false },  /* app-layer mission */
  /* --- Deck drivers (flow / vl53l8cx / microSD) --- */
  { "FLOW",       FLOW_TASK_NAME,          0, 0, 0, false },  /* PMW3901 optical flow */
  { "ZRANGER2",   ZRANGER2_TASK_NAME,      0, 0, 0, false },  /* VL53L1x on flow deck */
  { "VL53L8CX",   "vl53l8cx",              0, 0, 0, false },  /* multizone ToF deck */
  { "USDLOG",     USDLOG_TASK_NAME,        0, 0, 0, false },  /* SD log sampler */
  { "USDWRITE",   USDWRITE_TASK_NAME,      0, 0, 0, false },  /* SD card writer */
  /* --- System / comms --- */
  { "SYSTEM",     SYSTEM_TASK_NAME,        0, 0, 0, false },
  { "CRTP_TX",    CRTP_TX_TASK_NAME,       0, 0, 0, false },
  { "CRTP_RX",    CRTP_RX_TASK_NAME,       0, 0, 0, false },
  { "SYSLINK",    SYSLINK_TASK_NAME,       0, 0, 0, false },
  { "USBLINK",    USBLINK_TASK_NAME,       0, 0, 0, false },
  { "LOG",        LOG_TASK_NAME,           0, 0, 0, false },
  { "PARAM",      PARAM_TASK_NAME,         0, 0, 0, false },
  { "MEM",        MEM_TASK_NAME,           0, 0, 0, false },
  { "WORKER",     WORKER_TASK_NAME,        0, 0, 0, false },
  { "PLATFORM",   PLATFORM_SRV_TASK_NAME,  0, 0, 0, false },
  { "CRTP_SRV",   CRTP_SRV_TASK_NAME,      0, 0, 0, false },
  /* --- Idle (for reference / CPU headroom) --- */
  { "IDLE",       "IDLE",                  0, 0, 0, false },
};

#define NUM_ENTRIES (sizeof(entries) / sizeof(entries[0]))
/* 3デッキ構成では実タスク数が32を超えるため、uxTaskGetSystemState が一部の
 * タスク(STABILIZER 等)を取りこぼしていた。余裕を持って48にする。 */
#define TASK_MAX_COUNT 48

static bool initialized = false;
static uint16_t periodMs = 1000;          /* sampling interval (param) */

static uint32_t totalDeltaUs = 0;         /* sum of all tasks' CPU time */
static uint32_t intervalUs = 0;           /* measured wall-clock interval */
static float cpuLoadPct = 0.0f;           /* 100 * (1 - idle/interval) */

static uint32_t prevSampleUs = 0;
static uint32_t prevTotalRunTime = 0;

/* --- メモリ使用状況 --- */
static uint32_t heapFreeB = 0;      /* FreeRTOS ヒープの空き [byte] */
static uint32_t heapMinFreeB = 0;   /* 起動以降の空きの最小値 [byte] */
static uint16_t stackMinFreeB = 0;  /* 全タスク中で最も余裕の無いスタック [byte] */
static uint8_t stackMinTaskIdx = 0; /* そのタスクの entries[] 上の番号 */

NO_DMA_CCM_SAFE_ZERO_INIT static TaskStatus_t taskStats[TASK_MAX_COUNT];
static StaticTimer_t timerBuffer;
static xTimerHandle timer;

static taskEntry_t* findEntry(const char* taskName)
{
  for (uint32_t i = 0; i < NUM_ENTRIES; i++) {
    if (strcmp(entries[i].taskName, taskName) == 0) {
      return &entries[i];
    }
  }
  return NULL;
}

static void timerHandler(xTimerHandle t)
{
  uint32_t totalRunTime = 0;
  uint32_t now = usecTimestamp();

  uint32_t taskCount = uxTaskGetSystemState(taskStats, TASK_MAX_COUNT, &totalRunTime);
  ASSERT(taskCount <= TASK_MAX_COUNT);

  intervalUs = now - prevSampleUs;
  prevSampleUs = now;

  uint32_t idleDelta = 0;
  totalDeltaUs = 0;

  for (uint32_t i = 0; i < NUM_ENTRIES; i++) {
    entries[i].seen = false;
  }

  for (uint32_t i = 0; i < taskCount; i++) {
    TaskStatus_t* s = &taskStats[i];
    taskEntry_t* e = findEntry(s->pcTaskName);
    if (e) {
      uint32_t delta = s->ulRunTimeCounter - e->prevRunTime;
      e->prevRunTime = s->ulRunTimeCounter;
      e->deltaUs = delta;
      /* usStackHighWaterMark は「起動以降に残った最小のスタック」をワード数で
       * 返す。単調非増加なので、時間とともに下がっていく = そのタスクがより
       * 深い経路を通ったことを意味する。0 になるとオーバーフロー寸前。 */
      e->stackFreeB = (uint16_t)(s->usStackHighWaterMark * sizeof(StackType_t));
      e->seen = true;
      totalDeltaUs += delta;
      if (strcmp(s->pcTaskName, "IDLE") == 0) {
        idleDelta = delta;
      }
    }
  }

  /* Any task we track but that vanished this round -> report 0. */
  for (uint32_t i = 0; i < NUM_ENTRIES; i++) {
    if (!entries[i].seen) {
      entries[i].deltaUs = 0;
    }
  }

  /* 一番余裕の無いスタックを探す (存在するタスクのみ)。 */
  stackMinFreeB = 0xFFFF;
  stackMinTaskIdx = 0;
  for (uint32_t i = 0; i < NUM_ENTRIES; i++) {
    if (entries[i].seen && entries[i].stackFreeB < stackMinFreeB) {
      stackMinFreeB = entries[i].stackFreeB;
      stackMinTaskIdx = (uint8_t)i;
    }
  }
  if (stackMinFreeB == 0xFFFF) {
    stackMinFreeB = 0;
  }

  /* FreeRTOS ヒープ (xTaskCreate 等の動的確保。静的確保分は含まない)。 */
  heapFreeB = (uint32_t)xPortGetFreeHeapSize();
  heapMinFreeB = (uint32_t)xPortGetMinimumEverFreeHeapSize();

  /* Overall CPU load from the run-time counter total (includes untracked
   * tasks too, which is what we want for a true utilisation figure). */
  uint32_t totalDelta = totalRunTime - prevTotalRunTime;
  prevTotalRunTime = totalRunTime;
  if (totalDelta > 0) {
    cpuLoadPct = 100.0f * (float)(totalDelta - idleDelta) / (float)totalDelta;
  }
}

void taskLoadInit(void)
{
  ASSERT(!initialized);

  prevSampleUs = usecTimestamp();
  timer = xTimerCreateStatic("taskLoadTimer", M2T(periodMs), pdTRUE, NULL,
                             timerHandler, &timerBuffer);
  xTimerStart(timer, 100);

  initialized = true;
}

/**
 * @brief Per-task CPU execution time and overall load.
 *
 * Each taskload.<TASK> is the microseconds of CPU time that task consumed
 * during the last sampling interval (default 1000 ms). Normalise with
 * taskload.period (actual interval, us) for an exact duty ratio.
 */
LOG_GROUP_START(taskload)
LOG_ADD(LOG_FLOAT,  cpuLoad,  &cpuLoadPct)
LOG_ADD(LOG_UINT32, total,    &totalDeltaUs)
LOG_ADD(LOG_UINT32, period,   &intervalUs)
LOG_ADD(LOG_UINT32, STABILIZER, &entries[0].deltaUs)
LOG_ADD(LOG_UINT32, KALMAN,     &entries[1].deltaUs)
LOG_ADD(LOG_UINT32, SENSORS,    &entries[2].deltaUs)
LOG_ADD(LOG_UINT32, PWRMGNT,    &entries[3].deltaUs)
LOG_ADD(LOG_UINT32, CMDHL,      &entries[4].deltaUs)
LOG_ADD(LOG_UINT32, APP,        &entries[5].deltaUs)
LOG_ADD(LOG_UINT32, FLOW,       &entries[6].deltaUs)
LOG_ADD(LOG_UINT32, ZRANGER2,   &entries[7].deltaUs)
LOG_ADD(LOG_UINT32, VL53L8CX,   &entries[8].deltaUs)
LOG_ADD(LOG_UINT32, USDLOG,     &entries[9].deltaUs)
LOG_ADD(LOG_UINT32, USDWRITE,   &entries[10].deltaUs)
LOG_ADD(LOG_UINT32, SYSTEM,     &entries[11].deltaUs)
LOG_ADD(LOG_UINT32, CRTP_TX,    &entries[12].deltaUs)
LOG_ADD(LOG_UINT32, CRTP_RX,    &entries[13].deltaUs)
LOG_ADD(LOG_UINT32, SYSLINK,    &entries[14].deltaUs)
LOG_ADD(LOG_UINT32, USBLINK,    &entries[15].deltaUs)
LOG_ADD(LOG_UINT32, LOGT,       &entries[16].deltaUs)
LOG_ADD(LOG_UINT32, PARAM,      &entries[17].deltaUs)
LOG_ADD(LOG_UINT32, MEM,        &entries[18].deltaUs)
LOG_ADD(LOG_UINT32, WORKER,     &entries[19].deltaUs)
LOG_ADD(LOG_UINT32, PLATFORM,   &entries[20].deltaUs)
LOG_ADD(LOG_UINT32, CRTP_SRV,   &entries[21].deltaUs)
LOG_ADD(LOG_UINT32, IDLE,       &entries[22].deltaUs)
LOG_GROUP_STOP(taskload)

/**
 * @brief メモリ使用状況。
 *
 * ビルド時に出る "RAM 96%" は静的確保 (bss+data) の割合で、実行中は変化しない。
 * 実行時に変化するのは以下の2つ:
 *   - FreeRTOS ヒープ (xTaskCreate 等の動的確保)。静的確保のタスクは含まない。
 *   - 各タスクのスタック残量。usStackHighWaterMark は「起動以降の最小残量」なので
 *     単調に減っていく。0 に近づくとスタックオーバーフローの危険がある。
 * memload.stackMin が全タスク中の最小残量、stackMinId がそのタスク番号
 * (taskload の LOG 変数の並び順と同じ 0=STABILIZER, 1=KALMAN, ...)。
 */
LOG_GROUP_START(memload)
LOG_ADD(LOG_UINT32, heapFree,   &heapFreeB)
LOG_ADD(LOG_UINT32, heapMin,    &heapMinFreeB)
LOG_ADD(LOG_UINT16, stackMin,   &stackMinFreeB)
LOG_ADD(LOG_UINT8,  stackMinId, &stackMinTaskIdx)
LOG_GROUP_STOP(memload)

/**
 * @brief 各タスクのスタック残量 [byte] (起動以降の最小値)。
 * 全部を記録すると log 変数の上限 (128/イベント) を圧迫するので、
 * 必要なものだけ config.txt に並べればよい。
 */
LOG_GROUP_START(taskstack)
LOG_ADD(LOG_UINT16, STABILIZER, &entries[0].stackFreeB)
LOG_ADD(LOG_UINT16, KALMAN,     &entries[1].stackFreeB)
LOG_ADD(LOG_UINT16, SENSORS,    &entries[2].stackFreeB)
LOG_ADD(LOG_UINT16, PWRMGNT,    &entries[3].stackFreeB)
LOG_ADD(LOG_UINT16, CMDHL,      &entries[4].stackFreeB)
LOG_ADD(LOG_UINT16, APP,        &entries[5].stackFreeB)
LOG_ADD(LOG_UINT16, FLOW,       &entries[6].stackFreeB)
LOG_ADD(LOG_UINT16, ZRANGER2,   &entries[7].stackFreeB)
LOG_ADD(LOG_UINT16, VL53L8CX,   &entries[8].stackFreeB)
LOG_ADD(LOG_UINT16, USDLOG,     &entries[9].stackFreeB)
LOG_ADD(LOG_UINT16, USDWRITE,   &entries[10].stackFreeB)
LOG_ADD(LOG_UINT16, SYSTEM,     &entries[11].stackFreeB)
LOG_ADD(LOG_UINT16, CRTP_TX,    &entries[12].stackFreeB)
LOG_ADD(LOG_UINT16, CRTP_RX,    &entries[13].stackFreeB)
LOG_ADD(LOG_UINT16, SYSLINK,    &entries[14].stackFreeB)
LOG_ADD(LOG_UINT16, USBLINK,    &entries[15].stackFreeB)
LOG_ADD(LOG_UINT16, LOGT,       &entries[16].stackFreeB)
LOG_ADD(LOG_UINT16, PARAM,      &entries[17].stackFreeB)
LOG_ADD(LOG_UINT16, MEM,        &entries[18].stackFreeB)
LOG_ADD(LOG_UINT16, WORKER,     &entries[19].stackFreeB)
LOG_ADD(LOG_UINT16, PLATFORM,   &entries[20].stackFreeB)
LOG_ADD(LOG_UINT16, CRTP_SRV,   &entries[21].stackFreeB)
LOG_ADD(LOG_UINT16, IDLE,       &entries[22].stackFreeB)
LOG_GROUP_STOP(taskstack)

PARAM_GROUP_START(taskload)
/**
 * @brief Sampling interval in milliseconds (applied at boot).
 */
PARAM_ADD(PARAM_UINT16, periodMs, &periodMs)
PARAM_GROUP_STOP(taskload)
