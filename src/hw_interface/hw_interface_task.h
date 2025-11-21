#ifndef HW_INTERFACE_TASK_H_ // [수정] 헤더 가드 변경
#define HW_INTERFACE_TASK_H_

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
    float angle_deg;    // 모터 피드백 각도(deg) (하드웨어에서 환산)
} ServoFeedback_t;


void vHwInterfaceTask(void *pvParameters);

#endif /* HW_INTERFACE_TASK_H_ */
