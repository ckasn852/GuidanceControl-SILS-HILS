#include "rx_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "lwip/sockets.h"

#include "../control/control_data.h" 			// 제어 데이터 헤더
#include "../control/control_queue.h" 			// 제어 데이터 큐
#include "../network_init/network_bootstrap.h"

char recv_buffer[MAX_BUFFER_LEN]; // 수신 버퍼
int n; // 수신 데이터 갯수

void rx_task(){
    // 제어 데이터
    control_data rx_from_unreal;

    // 네트워크 준비 대기
    while (udp_sock < 0) {
        printf("rx_task: waiting for udp_sock\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }

	while(1)
	    {
        	xil_printf("rx_task\r\n ");
		    // 언리얼 엔진으로 부터 데이터 수신 대기
		    n = recvfrom(udp_sock, recv_buffer, sizeof(recv_buffer) - 1, 0,
		        (struct sockaddr*)&client_addr, &client_addr_len);

			if (n < 0) {
				xil_printf("Error: recvfrom failed\r\n");
	            vTaskDelay(pdMS_TO_TICKS(10));
	            continue;
			}
			else{
				xil_printf("Data num : %d \r\n", n);

				recv_buffer[n] = '\0';

				memset(&rx_from_unreal, 0, sizeof(rx_from_unreal));

	            // 아주 단순 파서 예시 (진짜 문자열 형식에 맞게 조정 필요)
	            // 실제 언리얼 송신 포맷에 맞게 바꾸셔야 합니다.
	            sscanf(recv_buffer,
	                   "P : %f Y : %f x : %f y : %f d : %f",
	                   &rx_from_unreal.img_x,
	                   &rx_from_unreal.img_y,
	                   &rx_from_unreal.distance,
	                   &rx_from_unreal.yaw_rate,
	                   &rx_from_unreal.pitch_rate);

	            // 파싱된 값 출력
	            printf(
	                "rx_task : x: %.3f y: %.3f dist: %.3f yaw: %.3f pitch: %.3f\r\n",
	                rx_from_unreal.img_x,
	                rx_from_unreal.img_y,
	                rx_from_unreal.distance,
	                rx_from_unreal.yaw_rate,
	                rx_from_unreal.pitch_rate
	            );

	            // 큐로 제어 데이터 push
	            if (xQueueSend(xControlQueue, &rx_from_unreal, 0) != pdPASS)
	            {
	                // 큐가 가득 찼다면 가장 오래된 게 의미 없을 가능성이 높으므로
	                // 오래된 걸 버리고 새 걸 넣고 싶다면 xQueueOverwrite (큐를 QueueSet 아닌 StreamBuffer로 만드는 방식도 가능)
	                printf("rx_task: queue full\n");
	            }
			}
	        vTaskDelay(pdMS_TO_TICKS(1000)); // 예시
	    }
}
