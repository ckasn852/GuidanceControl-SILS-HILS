#include "rx_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/sockets.h"
#include "semphr.h"
#include <stdbool.h>
#include <string.h>

#include "../control/control_data.h"             // 제어 데이터 헤더
#include "../control/control_queue.h"            // 제어 데이터 큐
#include "../network_init/network_bootstrap.h"

#include "queue.h"
#include "../control/pid_tuning_mode.h"

// main.c 에 정의된 뮤텍스 참조
extern SemaphoreHandle_t printMutex;

char recv_buffer[MAX_BUFFER_LEN];        // 수신 버퍼
int n;                                   // 수신 데이터 갯수
int sim_state = 0;                       // 시뮬레이션 상태


void rx_task()
{
    // 제어 데이터
    control_data rx_from_unreal;

    // 네트워크 준비 대기
    while (udp_sock < 0) {
        if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
            printf("rx_task: waiting for udp_sock\n");
            xSemaphoreGive(printMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Non-blocking 모드 설정 (LwIP 방식인 ioctlsocket 사용)
    // 1 = Non-blocking, 0 = Blocking
    u32_t mode = 1;
    ioctlsocket(udp_sock, FIONBIO, &mode);

    while(1)
    {
        // 버퍼가 빌 때까지 반복 수신하여 최신 1개 만 큐에 전달
        bool got_data = false;
        int last_packet_len = 0;

        // 소켓 버퍼 비우기 (Socket Draining)
        while (1) {
            n = recvfrom(udp_sock, recv_buffer, sizeof(recv_buffer) - 1, 0,
                         (struct sockaddr*)&client_addr, &client_addr_len);

            if (n > 0) {
                // 데이터를 받음 -> 계속 루프를 돌며 다음 데이터 확인 (덮어쓰기)
                last_packet_len = n;
                got_data = true;
            } else {
                // 수신 데이터 없으면 루프 탈출
                break;
            }
        }

        // 데이터가 하나도 안 왔으면(타임아웃/데이터 없음) 다음 주기로
        if (!got_data) {
            // 수신 데이터 없을 때 짧게 양보
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        recv_buffer[last_packet_len] = '\0';
        memset(&rx_from_unreal, 0, sizeof(rx_from_unreal));

        // 1. Control 플래그 포맷 처리: "Control: %d"
        int ctrl;
        if (sscanf(recv_buffer, "Control: %d", &ctrl) == 1) {
            PIDUpdate_t upd = {0};
            upd.tuning_active = ctrl;
            upd.has_params = 0;
            xQueueSend(xPidUpdateQueue, &upd, 0);
            if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
                printf("[rx_task][MOD] Control flag received: %d\r\n", ctrl);
                xSemaphoreGive(printMutex);
            }
            continue;
        }

        // 2. PID 파라미터 포맷 처리: "P: %f I: %f D: %f"
        float P, I, D;
        if (sscanf(recv_buffer, "P: %f I: %f D: %f", &P, &I, &D) == 3) {
            PIDUpdate_t upd = {0};
            upd.tuning_active = 0;
            upd.Kp = P; upd.Ki = I; upd.Kd = D;
            upd.has_params = 1;
            xQueueSend(xPidUpdateQueue, &upd, 0);
            if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
                printf("[rx_task][MOD] PID params received: P=%.3f I=%.3f D=%.3f\r\n", P, I, D);
                xSemaphoreGive(printMutex);
            }
            continue;
        }

        // 언리얼 데이터 수신
        int parsed = sscanf(recv_buffer,
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

        if (parsed < 1) {
            if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
                printf("rx_task: bad format (parsed=%d) buf=[%s]\n", parsed, recv_buffer);
                xSemaphoreGive(printMutex);
            }
            continue;
        }

        // xQueueOverwrite 사용 (항상 최신 데이터 유지)
        xQueueOverwrite(xControlQueue, &rx_from_unreal);
    }
}
