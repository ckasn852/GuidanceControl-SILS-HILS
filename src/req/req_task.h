#ifndef REQ_TASK_H
#define REQ_TASK_H

// req_task 함수 선언
// FreeRTOS xTaskCreate에서 호출하기 위한 태스크 엔트리 포인트입니다.
void req_task(void *pvParameters);

#endif /* REQ_TASK_H */
