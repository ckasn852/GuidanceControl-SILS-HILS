// control_queue.c
#include "control_queue.h"

QueueHandle_t xControlQueue = NULL;
QueueHandle_t xControlOutQueue = NULL;

void control_queue_init(void)
{
    // 입력 큐: 타겟, 동체 상태 데이터
    xControlQueue = xQueueCreate(100, sizeof(control_data));

    // 출력 큐: 날개 각도 제어 명령
    xControlOutQueue = xQueueCreate(100, sizeof(control_output));
}
