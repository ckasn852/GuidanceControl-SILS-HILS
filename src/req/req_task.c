#include "req_task.h"
#include "lwip/sockets.h"
#include "../network_init/network_bootstrap.h" // udp_sock, client_addr
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>                 // [MOD] strlen 사용을 위해 추가

// main.c에 정의된 소켓 뮤텍스 참조
extern SemaphoreHandle_t socketMutex;
// [MOD] 로그 출력을 위한 printMutex extern
extern SemaphoreHandle_t printMutex;

void req_task(void *pvParameters) {

    (void)pvParameters;

    // 요청 패킷 데이터
    char req_packet[] = "DATA_REQ";

    // 주기 확인용 변수
    static TickType_t last_tick = 0;
    static uint32_t send_cnt = 0;

    while(1) {
        // 1. 타이머 ISR에 의해 20ms마다 깨어남 (가장 높은 우선순위 권장)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 2. 언리얼에 요청 패킷 전송
        // [중요] tx_task와 소켓을 공유하므로 뮤텍스 사용
        if (xSemaphoreTake(socketMutex, portMAX_DELAY) == pdTRUE) {
            sendto(udp_sock, req_packet, strlen(req_packet), 0,
                   (struct sockaddr*)&client_addr, client_addr_len);
            xSemaphoreGive(socketMutex);
        }

        // 보낸 시점의 Tick을 이용해서 20ms 주기 로그 확인
        TickType_t now = xTaskGetTickCount();
        if (last_tick != 0) {
            TickType_t diff_tick = now - last_tick;
            uint32_t diff_ms = diff_tick * portTICK_PERIOD_MS;
            send_cnt++;

            // 1초에 한 번씩 출
            if ((send_cnt % 50u) == 0u) {
                if (xSemaphoreTake(printMutex, 0) == pdTRUE) {
                    printf("[req] send_cnt=%lu, period=%lu ms\r\n",
                           (unsigned long)send_cnt,
                           (unsigned long)diff_ms);
                    xSemaphoreGive(printMutex);
                }
            }
        }
        last_tick = now;

        // 할 일 끝, 다음 주기까지 대기
    }
}
