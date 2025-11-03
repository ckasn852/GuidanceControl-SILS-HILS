// File: motor/motor_driver_task.h
// (±‚¡∏ motor/control_task.h)

#ifndef MOTOR_DRIVER_TASK_H_
#define MOTOR_DRIVER_TASK_H_

#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t xAngleQueue;

typedef struct {
    int motor_id;
    float angle;
} ServoCommand_t;




void vMotorDriverTask(void *pvParameters);

#endif /* MOTOR_DRIVER_TASK_H_ */
