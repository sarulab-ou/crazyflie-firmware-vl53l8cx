/**
 * taskload.h - Per-task CPU execution-time logger.
 *
 * Periodically samples every FreeRTOS task's cumulative run-time counter
 * (microseconds, from usecTimestamp) and exposes the CPU time each task
 * consumed during the last interval as LOG variables, so they can be
 * recorded to the microSD-card deck. See taskload.c for the metric.
 */
#ifndef TASKLOAD_H_
#define TASKLOAD_H_

void taskLoadInit(void);

#endif /* TASKLOAD_H_ */
