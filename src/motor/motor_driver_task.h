#ifndef MOTOR_DRIVER_TASK_H_
#define MOTOR_DRIVER_TASK_H_

#include "FreeRTOS.h"
#include "queue.h"

extern QueueHandle_t xAngleQueue;

typedef struct {
    int motor_id;
    float angle;
} ServoCommand_t;

// 모터 피드백 전송용 큐
extern QueueHandle_t xAngleFbQueue;

typedef struct {
    int motor_id;
    float angle_deg;	// 모터 피드백 각도(deg) (하드웨어에서 환산)
} ServoFeedback_t;

void vMotorDriverTask(void *pvParameters);

#endif /* MOTOR_DRIVER_TASK_H_ */
