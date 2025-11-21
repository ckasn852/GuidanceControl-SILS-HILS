// control_queue.h
#ifndef CONTROL_QUEUE_H
#define CONTROL_QUEUE_H

#include "FreeRTOS.h"
#include "queue.h"

#include "control_data.h"
#include "control_output.h"
#include "control_output_hw.h"

extern QueueHandle_t xControlQueue;        // rx_task -> control_task
extern QueueHandle_t xControlOutQueue;     // control_task -> tx_task
extern QueueHandle_t xRealDataQueue;     // hw_interface_task -> tx_task

void control_queue_init(void);

#endif
