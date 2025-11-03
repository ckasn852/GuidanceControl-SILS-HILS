#include "rx_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/sockets.h"
#include <stdbool.h>
#include "semphr.h"

#include "../control/control_data.h" 			// 제어 데이터 헤더
#include "../control/control_queue.h" 			// 제어 데이터 큐
#include "../network_init/network_bootstrap.h"

// main.c 에 정의된 뮤텍스 참조
extern SemaphoreHandle_t printMutex;

char recv_buffer[MAX_BUFFER_LEN];	 	// 수신 버퍼
int n;									// 수신 데이터 갯수

void rx_task(){
    // 제어 데이터
    control_data rx_from_unreal;
    float pitch_wing_deg_rx, yaw_wing_deg_rx;

    bool sim_state = 0;

    // 네트워크 준비 대기
    while (udp_sock < 0) {
    	if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
    	            printf("rx_task: waiting for udp_sock\n");
    	            xSemaphoreGive(printMutex);
    	        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

	while(1)
	    {
		    // 언리얼 엔진으로 부터 데이터 수신 대기
		    n = recvfrom(udp_sock, recv_buffer, sizeof(recv_buffer) - 1, 0,
		        (struct sockaddr*)&client_addr, &client_addr_len);

			if (n < 0) {
				if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
					xil_printf("Error: recvfrom failed\r\n");
					xSemaphoreGive(printMutex);
				}
	            vTaskDelay(pdMS_TO_TICKS(10));
	            continue;
			}
			else{
				recv_buffer[n] = '\0';

				memset(&rx_from_unreal, 0, sizeof(rx_from_unreal));

	            // 언리얼 데이터 수신 : (Pitch, Yaw)동체 각속도, (x, y)표적 좌표, (Pitch, Yaw) 날개 각도, 시뮬레이션 상태
	            sscanf(recv_buffer,
	                   "PV: %f YV: %f x: %f y: %f d: %f PW: %f YW: %f State: %d P: %f Y: %f",
	                   &rx_from_unreal.pitch_rate,
	                   &rx_from_unreal.yaw_rate,
	                   &rx_from_unreal.img_x,
	                   &rx_from_unreal.img_y,
					   &rx_from_unreal.distance,
	                   &rx_from_unreal.pitch_wing_deq,
					   &rx_from_unreal.yaw_wing_deq,
					   &rx_from_unreal.sim_state,
					   &rx_from_unreal.pitch,
					   &rx_from_unreal.yaw
					   );

				// 파싱된 값 출력(뮤텍스로 보호)
				if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
					printf(
						"rx_task received data : PV: %f YV: %f x: %f y: %f d: %f PW: %f YW: %f State: %d P: %f Y: %f\r\n\n",
						   rx_from_unreal.pitch_rate,
						   rx_from_unreal.yaw_rate,
						   rx_from_unreal.img_x,
						   rx_from_unreal.img_y,
						   rx_from_unreal.distance,
						   rx_from_unreal.pitch_wing_deq,
						   rx_from_unreal.yaw_wing_deq,
						   rx_from_unreal.sim_state,
						   rx_from_unreal.pitch,
						   rx_from_unreal.yaw
					);
					xSemaphoreGive(printMutex);
				}

	            // 큐로 제어 데이터 push
	            if (xQueueSend(xControlQueue, &rx_from_unreal, pdMS_TO_TICKS(0)) != pdPASS)
	            {
	                // 큐가 가득 찼다면 가장 오래된 게 의미 없을 가능성이 높으므로
	                // 오래된 걸 버리고 새 걸 넣고 싶다면 xQueueOverwrite (큐를 QueueSet 아닌 StreamBuffer로 만드는 방식도 가능)
	                // printf("rx_task: queue full\n");
	            }
			}
	    }
}
