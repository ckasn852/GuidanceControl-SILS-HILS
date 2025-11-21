#include "lwip/sockets.h"
#include "../network_init/network_bootstrap.h"
#include "tx_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdbool.h>
#include <string.h> // memset
#include "../control/control_data.h"
#include "../control/control_queue.h"
#include "../control/control_output.h"
#include "../control/control_output_hw.h"

// 태스크 시간 측정
#include "xtime_l.h"
#include "../sys_stat/system_stats.h"

// 디버깅 뮤텍스, 소켓 뮤텍스
extern SemaphoreHandle_t printMutex;
extern SemaphoreHandle_t socketMutex;

// 모터 피드백 큐
#include "../hw_interface/hw_interface_task.h"

void tx_task(){
    // 태스크 수행시간 측정용 변수
    XTime tStart, tEnd;

    control_output sim_data;       // 언리얼로 보내는 가상 데이터
    real_sensor_data real_data;    // 하드웨어로 보내는 센서 데이터

    // 초기화
    memset(&real_data, 0, sizeof(real_data));

    int data_len = 0;
    char send_buffer[TX_BUFFER_LEN];
    // char* p = send_buffer; // [삭제] 사용하지 않는 변수 경고 해결

    uint32_t current_time_ms = 0;

    // UDP 송신 타임아웃(주기 침범 방지, 2ms)
    struct timeval stv = { .tv_sec = 0, .tv_usec = 2000 };
    setsockopt(udp_sock, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

    while(1)
    {
        // Control 태스크 대기
        if (xQueueReceive(xControlOutQueue, &sim_data, portMAX_DELAY) != pdTRUE) continue;

        // Hardware 태스크 확인 (Non-blocking)
        if (xQueueReceive(xRealDataQueue, &real_data, 0) == pdTRUE) {
             // 최신값 수신
        }

        /* 제어량 최종 송신 (시뮬레이션 진행중일 때만 송신) */
        if(sim_data.sim_state == 0){
        	/* 시간 측정 시작 (패킷 생성 및 송신) */
        	XTime_GetTime(&tStart);

        	// 큐에서 데이터를 받자마자 현재 시간을 ms 단위로 측정 (Tick 수 * Tick 당 ms)
            current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

            // 태스크 전체 실행 시간 합산
            float total_exec_time = g_time_rx_us + g_time_ctrl_us + g_time_hw_us + g_time_tx_us;

            // [수정] 송신 포맷에 실제 데이터 추가
            data_len = snprintf(send_buffer, sizeof(send_buffer),
                    "Pitch_Fin_Control_Amount: %.4f Yaw_Fin_Control_Amount: %.4f "	// Pitch,Yaw 날개 각도 제어량
                    "target_pitch: %.4f target_yaw: %.4f "							// Pitch,Yaw 목표 동체 각속도
                    "Pitch_Motor_Feedback: %.4f Yaw_Motor_Feedback: %.4f "			// Pitch,Yaw 날개 각속도 현재 상태
                    "Send_Time: %lu",												// 언리얼로 데이터를 보낸 시간
                    sim_data.pitch_fin_deg, sim_data.yaw_fin_deg,
                    sim_data.target_pitch, sim_data.target_yaw,
                    real_data.real_motor_angle[0], real_data.real_motor_angle[2],
                    current_time_ms);

//            // [수정] 송신 포맷에 실제 데이터 추가
//            data_len = snprintf(send_buffer, sizeof(send_buffer),
//                    "Pitch_Fin_Control_Amount: %.4f Yaw_Fin_Control_Amount: %.4f "	// Pitch,Yaw 날개 각도 제어량
//                    "target_pitch: %.4f target_yaw: %.4f "							// Pitch,Yaw 목표 동체 각속도
//                    "Pitch_Motor_Feedback: %.4f Yaw_Motor_Feedback: %.4f "			// Pitch,Yaw 날개 각속도 현재 상태
//                    "RealIMU_Pitch: %.4f RealIMU_Yaw: %.4f " 						// Pitch,Yaw IMU 데이터
//                    "Send_Time: %lu"												// 언리얼로 데이터를 보낸 시간
//					  "Task_Execution_time: %.1f",									// 태스크 총 수행 시간
//                    sim_data.pitch_fin_deg, sim_data.yaw_fin_deg,
//                    sim_data.target_pitch, sim_data.target_yaw,
//                    real_data.real_motor_angle[0], real_data.real_motor_angle[2],
//                    real_data.real_imu_pitch, real_data.real_imu_yaw, 		// IMU 데이터
//                    current_time_ms, total_exec_time);

            // 언리얼에 UDP 송신(소켓 뮤텍스 사용)
			if (xSemaphoreTake(socketMutex, portMAX_DELAY) == pdTRUE) {
				sendto(udp_sock, send_buffer, data_len, 0, (struct sockaddr*)&client_addr, client_addr_len);
				xSemaphoreGive(socketMutex);
			}

//			if (xSemaphoreTake(printMutex, 0) == pdTRUE) {
//				printf("[tx] ExecTime: %.1f us (SendTime: %lu)\r\n", total_exec_time, current_time_ms);
//				xSemaphoreGive(printMutex);
//			}

//			// 언리얼에 송신한 전체 문자열 출력(뮤텍스로 보호)
//			if (xSemaphoreTake(printMutex, 0) == pdTRUE) {
//				printf("[tx] send -> %s\r\n", send_buffer);
//				xSemaphoreGive(printMutex);
//			}

//			// 1초에 한 번만 출력(50Hz * 50 = 1초)
//			static int log_cnt = 0;
//			log_cnt++;
//
//			// 언리얼에 송신한 전체 문자열 출력(뮤텍스로 보호)
//			if (log_cnt >= 50) {
//				log_cnt = 0;
//				if (xSemaphoreTake(printMutex, 0) == pdTRUE) {
//					printf("[tx] send -> %s\r\n", send_buffer);
//					xSemaphoreGive(printMutex);
//				}
//			}

//            // 1초에 한 번만 출력하도록 변경 (50Hz * 50 = 1초)
//            static int log_cnt = 0;
//            log_cnt++;
//
//            if (log_cnt >= 50) {
//                log_cnt = 0;
//                if (xSemaphoreTake(printMutex, 0) == pdTRUE) { // 0: 기다리지 않고 가능할 때만 출력
//                     // 실수형 출력(%f)도 부하가 크므로 꼭 필요할 때만 사용
//                     printf("[tx] ExecTime: %.1f us (SendTime: %lu)\r\n", total_exec_time, current_time_ms);
//                     xSemaphoreGive(printMutex);
//                }
//            }

            /* 시간 측정 종료 및 기록 */
			XTime_GetTime(&tEnd);
			g_time_tx_us = (float)((double)(tEnd - tStart) / (COUNTS_PER_SECOND / 1000000.0));
        }
    }
}
