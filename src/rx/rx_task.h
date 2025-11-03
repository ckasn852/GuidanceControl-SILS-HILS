#ifndef RX_TASK_H
#define RX_TASK_H

#define MAX_BUFFER_LEN      256

extern char recv_buffer[MAX_BUFFER_LEN]; // t신 버퍼
extern int n; // 수신 데이터 갯수
void rx_task(); // rx_task 함수 선언

#endif
