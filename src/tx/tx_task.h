#ifndef TX_TASK_H
#define TX_TASK_H

#define MAX_BUFFER_LEN	1024

extern char send_buffer[MAX_BUFFER_LEN]; // 송신 버퍼
void tx_task(); // tx_task 함수 선언

#endif
