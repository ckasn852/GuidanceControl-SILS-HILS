#include "rx_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/sockets.h"
#include "semphr.h"
#include <stdbool.h>
#include <string.h>

#include "../control/control_data.h"
#include "../control/control_queue.h"
#include "../network_init/network_bootstrap.h"

#include "queue.h"
#include "../control/pid_tuning_mode.h"

// 태스크 시간 측정
#include "xtime_l.h"
#include "../sys_stat/system_stats.h"

extern SemaphoreHandle_t printMutex;

// [MOD] Req Task의 공유 변수 참조
extern volatile XTime g_shared_req_time;
extern volatile float g_shared_req_period;
// [DEL] 기존 전역변수 g_req_send_timestamp, g_req_pending, g_last_rtt_us 제거

char recv_buffer[MAX_BUFFER_LEN];
int n;
int sim_state = 0;

void rx_task()
{
    XTime tStart, tEnd;
    control_data rx_from_unreal;

    // 데이터 패킷 보존을 위한 별도 버퍼 추가
    // recv_buffer는 Control/PID 패킷에 의해 덮어씌워질 수 있으므로, 순수 데이터는 여기에 백업
    char latest_data_buffer[MAX_BUFFER_LEN];

    // 데이터 파싱 전 임시 저장할 시간 변수
    float temp_rtt_us = 0.0f;
    float temp_period_us = 0.0f;
    XTime temp_rx_finish = 0;

    while (udp_sock < 0) {
        if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
            printf("rx_task: waiting for udp_sock\n");
            xSemaphoreGive(printMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    u32_t mode = 1;
    ioctlsocket(udp_sock, FIONBIO, &mode);

    while(1)
    {
        bool got_new_sensor_data = false;

        // 소켓 드레인 루프
        while (1) {
            n = recvfrom(udp_sock, recv_buffer, sizeof(recv_buffer) - 1, 0,
                         (struct sockaddr*)&client_addr, &client_addr_len);

            if (n > 0) {
                recv_buffer[n] = '\0';

                // 데이터 수신 시점 즉시 기록 (RTT 계산용)
                XTime tRxNow;
                XTime_GetTime(&tRxNow);

                // RTT 계산: 공유 변수 사용
                if (g_shared_req_time != 0 && tRxNow > g_shared_req_time) {
                    temp_rtt_us = (float)(tRxNow - g_shared_req_time) / (float)(COUNTS_PER_SECOND / 1000000.0);
                } else {
                    temp_rtt_us = 0.0f;
                }

                // Req 주기 저장
                temp_period_us = g_shared_req_period;

                // Rx 종료 시간 저장 (Rx->Control 지연 계산용)
                temp_rx_finish = tRxNow;

                // 1. Control 패킷 우선 검사 및 즉시 처리
                int ctrl;
                if (strncmp(recv_buffer, "Control:", 8) == 0) {
                    if (sscanf(recv_buffer, "Control: %d", &ctrl) == 1) {
                        PIDUpdate_t upd = {0};
                        upd.tuning_active = ctrl;
                        upd.has_params = 0;
                        xQueueSend(xPidUpdateQueue, &upd, 0);
                        if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
                            printf("[rx_task] Control flag received: %d\r\n", ctrl);
                            xSemaphoreGive(printMutex);
                        }
                    }
                    continue; // Control 패킷 처리 완료, 다음 패킷 확인
                }

                // 2. PID 패킷 우선 검사 및 즉시 처리
                if (strncmp(recv_buffer, "P:", 2) == 0) {
                    float P, I, D;
                    if (sscanf(recv_buffer, "P: %f I: %f D: %f", &P, &I, &D) == 3) {
                        PIDUpdate_t upd = {0};
                        upd.tuning_active = 0;
                        upd.Kp = P; upd.Ki = I; upd.Kd = D;
                        upd.has_params = 1;
                        xQueueSend(xPidUpdateQueue, &upd, 0);
                        if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
                            printf("[rx_task] PID params received: P=%.3f I=%.3f D=%.3f\r\n", P, I, D);
                            xSemaphoreGive(printMutex);
                        }
                    }
                    continue; // PID 게인 패킷 처리 완료, 다음 패킷 확인
                }

                // 3. 일반 데이터 패킷 (PV...)
                // 데이터 패킷은 전용 버퍼에 '복사'해둡니다.
                // 이렇게 해야 나중에 Control 패킷이 또 들어와서 recv_buffer를 덮어써도 데이터가 안전
                memcpy(latest_data_buffer, recv_buffer, n + 1); // NULL 문자 포함 복사
                got_new_sensor_data = true;

            } else {
                break;
            }
        }

        if (!got_new_sensor_data) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        /* 여기서부터는 "가장 최신 데이터 패킷" 파싱 로직 */
        XTime_GetTime(&tStart);

        memset(&rx_from_unreal, 0, sizeof(rx_from_unreal));

        int parsed = sscanf(latest_data_buffer,
               "PV: %f YV: %f x: %d y: %d d: %f PW: %f YW: %f State: %d P: %f Y: %f",
               &rx_from_unreal.pitch_rate,
               &rx_from_unreal.yaw_rate,
               &rx_from_unreal.img_x,
               &rx_from_unreal.img_y,
               &rx_from_unreal.distance,
               &rx_from_unreal.pitch_wing_deq,
               &rx_from_unreal.yaw_wing_deq,
               &rx_from_unreal.sim_state,
               &rx_from_unreal.p,
               &rx_from_unreal.y
        );

        // [ADD] 측정된 시간 정보를 구조체에 담아 Control Task로 전달
        rx_from_unreal.req_period_us = temp_period_us;
        rx_from_unreal.rtt_us = temp_rtt_us;
        rx_from_unreal.rx_finish_time = temp_rx_finish;

        // 큐 전송
        xQueueOverwrite(xControlQueue, &rx_from_unreal);

        /* 실행 시간 측정 종료 & 통계 갱신 */
        XTime_GetTime(&tEnd);
        double exec_us = (double)(tEnd - tStart) / (COUNTS_PER_SECOND / 1000000.0);

        g_time_rx_us = (float)exec_us;
        g_sum_rx_us += exec_us;
        g_cnt_rx++;
        g_avg_rx_us = (g_cnt_rx > 0) ? (g_sum_rx_us / g_cnt_rx) : 0.0;
        if (exec_us > g_wcet_rx_us) {
            g_wcet_rx_us = exec_us;
        }
    }
}
