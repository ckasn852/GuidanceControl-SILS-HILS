// File: servo_task.h

#ifndef SERVO_TASK_H_
#define SERVO_TASK_H_

#include "FreeRTOS.h"
#include "queue.h"

// main.c에 정의된 앵글 큐에 접근하기 위한 extern 선언
extern QueueHandle_t xAngleQueue;

typedef struct {
    int motor_id;  // 0~5번 모터
    float angle;   // -90.0 ~ +90.0 각도
} ServoCommand_t;


/**
 * @brief Servo Control Task
 * xAngleQueue로부터 각도 값을 수신하여 서보모터를 제어합니다.
 * @param pvParameters FreeRTOS 태스크 파라미터 (사용 안 함)
 */
void vServoControlTask(void *pvParameters);

#endif /* SERVO_TASK_H_ */
