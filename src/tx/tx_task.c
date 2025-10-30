#include "lwip/sockets.h"
#include "../network_init/network_bootstrap.h"
#include "tx_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "../control/control_data.h" // 제어 데이터 헤더
#include "../control/control_queue.h" // 제어 데이터 큐

// main.c 에 정의된 뮤텍스 참조
extern SemaphoreHandle_t printMutex;

void tx_task(){

	control_output tx_to_unreal;					// 언리얼 송신 데이터
	int data_len = 0;								// 송신 데이터 길이
	char send_buffer[MAX_BUFFER_LEN]; 				// 송신 버퍼
	float pitch_fin_deg_out, yaw_fin_deg_out;		// Pitch, Yaw 방향 날개 제어각 출력

	while(1)
	{
		while(1)
		{
			// 큐에 데이터가 들어올 때까지 무한정 대기 (CPU 점유 0)
			if (xQueueReceive(xControlOutQueue, &tx_to_unreal, portMAX_DELAY) == pdTRUE)
			{
				// Pitch, Yaw 방향 날개 제어 명령 수신 값을 지역 변수에 할당
				pitch_fin_deg_out = tx_to_unreal.pitch_fin_deg;
				yaw_fin_deg_out = tx_to_unreal.yaw_fin_deg;

				// PID 날개 각도 제어량을 송신 버퍼에 저장
				data_len = snprintf(send_buffer, sizeof(send_buffer),
									"Pitch_Fin_Control_Amount: %.4f Yaw_Fin_Control_Amount: %.4f",
									pitch_fin_deg_out, yaw_fin_deg_out);

				// 언리얼에 UDP 송신
				sendto(udp_sock, send_buffer, data_len, 0, (struct sockaddr*)&client_addr, client_addr_len);

				// 언리얼에 송신한 PID 날개 각도 제어량 출력 확인(뮤텍스로 보호)
				if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
					printf("pitch_fin_deg_out: %.4f yaw_fin_deg_out: %.4f\r\n\n", pitch_fin_deg_out, yaw_fin_deg_out);
					xSemaphoreGive(printMutex);
				}

			}
		}
	}
}
