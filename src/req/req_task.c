#include "req_task.h"
#include "lwip/sockets.h"
#include "../network_init/network_bootstrap.h" // udp_sock, client_addr
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>                 // strlen 사용
#include "xtime_l.h"                // 시간 측정
#include "../sys_stat/system_stats.h"  // 통계 전역변수

// main.c에 정의된 소켓 뮤텍스 참조
extern SemaphoreHandle_t socketMutex;
// 로그 출력을 위한 printMutex extern
extern SemaphoreHandle_t printMutex;

// Rx Task와 공유할 타임스탬프 변수 (큐 없음)
volatile XTime g_shared_req_time = 0;
volatile float g_shared_req_period = 0.0f;

void req_task(void *pvParameters) {

    (void)pvParameters;

    // 요청 패킷 데이터
    char req_packet[] = "DATA_REQ";

    // 주기 확인용 변수
    static TickType_t last_tick = 0;
    static uint32_t send_cnt = 0;

    // 수행시간 측정 변수
    XTime tStart, tEnd;

    // 주기 측정용 이전 시작 시간
    static XTime tStartPrev = 0;

    while(1) {
        // 1. 타이머 ISR에 의해 20ms마다 깨어남 (가장 높은 우선순위 권장)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 시간 측정 시작 */
        XTime_GetTime(&tStart);

        // 1. req_task 송신 간격 계산 (이번 시작 시간 - 이전 시작 시간)
        if (tStartPrev != 0) {
            uint64_t diff = tStart - tStartPrev;
            // 기존 전역 g_req_period_us 대신 공유 변수에 저장
            g_shared_req_period = (float)diff / (float)(COUNTS_PER_SECOND / 1000000.0f);
        }
        tStartPrev = tStart;

        // 2. 언리얼에 요청 패킷 전송
        // tx_task와 소켓을 공유하므로 뮤텍스 사용
        if (xSemaphoreTake(socketMutex, portMAX_DELAY) == pdTRUE) {

            // RTT 측정 시작: 송신 시각 기록
            // 기존 g_req_send_timestamp / g_req_pending 로직 제거하고 단순 공유 변수 업데이트
            g_shared_req_time = tStart;

            sendto(udp_sock, req_packet, strlen(req_packet), 0,
                   (struct sockaddr*)&client_addr, client_addr_len);
            xSemaphoreGive(socketMutex);
        }

        /* 시간 측정 종료 및 통계 갱신 */
        XTime_GetTime(&tEnd);
        double exec_us = (double)(tEnd - tStart) / (COUNTS_PER_SECOND / 1000000.0);

        g_time_req_us = (float)exec_us;
        g_sum_req_us += exec_us;
        g_cnt_req++;
        g_avg_req_us = (g_cnt_req > 0) ? (g_sum_req_us / g_cnt_req) : 0.0;
        if (exec_us > g_wcet_req_us) {
            g_wcet_req_us = exec_us;
        }

        // 보낸 시점의 Tick을 이용해서 20ms 주기 로그 확인
        TickType_t now = xTaskGetTickCount();
        if (last_tick != 0) {
            TickType_t diff_tick = now - last_tick;
            uint32_t diff_ms = diff_tick * portTICK_PERIOD_MS;
            send_cnt++;
            // (기존 주석 처리된 로그 코드 생략됨)
        }
        last_tick = now;

        // 할 일 끝, 다음 주기까지 대기
    }
}
