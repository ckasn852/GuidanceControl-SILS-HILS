#include "rx_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/sockets.h"
#include "semphr.h"
#include <stdbool.h>

#include "../control/control_data.h"             // 제어 데이터 헤더
#include "../control/control_queue.h"            // 제어 데이터 큐
#include "../network_init/network_bootstrap.h"

#include "queue.h"
#include "../control/pid_tuning_mode.h"

// main.c 에 정의된 뮤텍스 참조
extern SemaphoreHandle_t printMutex;

char recv_buffer[MAX_BUFFER_LEN];        // 수신 버퍼
int n;                                   // 수신 데이터 갯수
int sim_state = 0;						// 시뮬레이션 상태


void rx_task()
{
    // 제어 데이터
    control_data rx_from_unreal;
    float pitch_wing_deg_rx, yaw_wing_deg_rx;
    uint32_t current_time_ms = 0;                    // 송신 시간을 저장할 변수

    // 네트워크 준비 대기
    while (udp_sock < 0) {
        if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
            printf("rx_task: waiting for udp_sock\n");
            xSemaphoreGive(printMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // recv 타임아웃 1ms 설정
    struct timeval tv = { .tv_sec = 0, .tv_usec = 1000 };
    setsockopt(udp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    while(1)
    {
        // 버퍼가 빌 때까지 반복 수신하여 최신 1개 만 큐에 전달
        bool got_data = false;

		// 언리얼 엔진으로 부터 데이터 수신 대기 (타임아웃 1ms)
		n = recvfrom(udp_sock, recv_buffer, sizeof(recv_buffer) - 1, 0,
					 (struct sockaddr*)&client_addr, &client_addr_len);

		if (n < 0) continue; // 더 이상 읽을 데이터 없음(타임아웃)

		// 언리얼에서 데이터를 받자마자 현재 시간을 ms 단위로 측정 (Tick 수 * Tick 당 ms)
		current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

//		// 언리얼 데이터 수신 시간 출력
//		if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
//			printf("rx_task : Time: %lu ms\r\n\n", current_time_ms);
//			xSemaphoreGive(printMutex);
//		}

		recv_buffer[n] = '\0';
		memset(&rx_from_unreal, 0, sizeof(rx_from_unreal));

        // 1. Control 플래그 포맷 처리: "Control: %d" (0=튜닝 적용중, 1=미적용중)
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
            continue;                                               // 다음 루프
        }

        // 2. PID 파라미터 포맷 처리: "P: %f I: %f D: %f"
        float P, I, D;
        if (sscanf(recv_buffer, "P: %f I: %f D: %f", &P, &I, &D) == 3) {
            PIDUpdate_t upd = {0};
            upd.tuning_active = 0;  // 파라미터는 "튜닝 적용중" 컨텍스트에서 유효
            upd.Kp = P; upd.Ki = I; upd.Kd = D;
            upd.has_params = 1;
            xQueueSend(xPidUpdateQueue, &upd, 0);
            if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
                printf("[rx_task][MOD] PID params received: P=%.3f I=%.3f D=%.3f\r\n", P, I, D);
                xSemaphoreGive(printMutex);
            }
            continue;
        }

		// 언리얼 데이터 수신 : (Pitch, Yaw)동체 각속도, (x, y)표적 좌표, (Pitch, Yaw) 날개 각도, 시뮬레이션 상태
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
			continue; // 파싱 실패 샘플은 버림
		}

		got_data = true;

//		// 파싱된 값 출력(뮤텍스로 보호)
//		if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
//			printf(
//				"rx_task received data : PV: %f YV: %f x: %d y: %d d: %f PW: %f YW: %f State: %d P: %f Y: %f\r\n\n",
//				rx_from_unreal.pitch_rate,
//				rx_from_unreal.yaw_rate,
//				rx_from_unreal.img_x,
//				rx_from_unreal.img_y,
//				rx_from_unreal.distance,
//				rx_from_unreal.pitch_wing_deq,
//				rx_from_unreal.yaw_wing_deq,
//				rx_from_unreal.sim_state,
//				rx_from_unreal.p,
//				rx_from_unreal.y
//			);
//			xSemaphoreGive(printMutex);
//		}

        if (!got_data) {
            // 수신 데이터 없을 때 짧게 양보
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // 큐로 제어 데이터 push (길이 1 큐를 권장. xQueueOverwrite 사용 시 아래 한 줄로 대체 가능)
        xQueueSend(xControlQueue, &rx_from_unreal, 0);
//        xQueueOverwrite(xControlQueue, &rx_from_unreal);
    }
}
