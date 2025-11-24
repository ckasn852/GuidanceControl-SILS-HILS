// control_queue.h
#ifndef CONTROL_QUEUE_H
#define CONTROL_QUEUE_H

#include "FreeRTOS.h"
#include "queue.h"

#include "control_data.h"
#include "control_output.h"
#include "control_output_hw.h" // [MOD] 헤더 다시 포함

extern QueueHandle_t xControlQueue;        // rx_task -> control_task
extern QueueHandle_t xControlOutQueue;     // control_task -> tx_task
extern QueueHandle_t xRealDataQueue;     // [MOD] hw_interface_task -> tx_task (복구)

void control_queue_init(void);

#endif
