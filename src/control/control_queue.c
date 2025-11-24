// control_queue.c
#include "control_queue.h"

QueueHandle_t xControlQueue = NULL;
QueueHandle_t xControlOutQueue = NULL;
QueueHandle_t xRealDataQueue = NULL; // [MOD] 변수 복구

void control_queue_init(void)
{
    // 입력 큐: 타겟, 동체 상태 데이터
    xControlQueue = xQueueCreate(1, sizeof(control_data));

    // 출력 큐: 날개 각도 제어 명령
    xControlOutQueue = xQueueCreate(1, sizeof(control_output));

    // IMU 데이터 전송을 위해 큐 생성
    xRealDataQueue = xQueueCreate(1, sizeof(real_sensor_data));
}
