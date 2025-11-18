#include "lwip/sockets.h"
#include "../network_init/network_bootstrap.h"
#include "tx_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <stdbool.h>
#include "../control/control_data.h" // 제어 데이터 헤더
#include "../control/control_queue.h" // 제어 데이터 큐

// main.c 에 정의된 뮤텍스 참조
extern SemaphoreHandle_t printMutex;

// 모터 피드백 큐
#include "../motor/motor_driver_task.h"

void tx_task(){

    control_output tx_to_unreal;                 // 언리얼 송신 데이터
    int data_len = 0;                            // 송신 데이터 길이
    char send_buffer[MAX_BUFFER_LEN];            // 송신 버퍼
    float pitch_fin_deg_out, yaw_fin_deg_out;    // Pitch, Yaw 방향 날개 제어각 출력
    float target_pitch, target_yaw;    			 // Pitch, Yaw 방향 목표 각속도(언리얼 송신용)
    uint32_t current_time_ms = 0;                // 송신 시간을 저장할 변수
    int sim_state = 0;

    // 피드백 최신값 캐시(모터쌍 평균)
    float fb_pitch_deg = 0.0f;
    float fb_yaw_deg   = 0.0f;

    // UDP 송신 타임아웃(주기 침범 방지, 2ms)
    struct timeval stv = { .tv_sec = 0, .tv_usec = 2000 };
    setsockopt(udp_sock, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

    while(1)
    {
    	if (xQueueReceive(xControlOutQueue, &tx_to_unreal, portMAX_DELAY) != pdTRUE) continue;
        {
            // Pitch, Yaw 방향 날개 제어 명령 수신 값을 지역 변수에 할당
            pitch_fin_deg_out = tx_to_unreal.pitch_fin_deg;
            yaw_fin_deg_out = tx_to_unreal.yaw_fin_deg;
            target_pitch = tx_to_unreal.target_pitch;
            target_yaw = tx_to_unreal.target_yaw;
            sim_state = tx_to_unreal.sim_state;

            // 모터 피드백 큐에서 최신값 비우며 집계(논블로킹)
            if (xAngleFbQueue) {
                ServoFeedback_t fb;
                float p0 = 0, p1 = 0, y2 = 0, y3 = 0;
                int   m_p0=0, m_p1=0, m_y2=0, m_y3=0;
                while (xQueueReceive(xAngleFbQueue, &fb, 0) == pdTRUE) {
                    if (fb.motor_id == 0) { p0 = fb.angle_deg; m_p0=1; }
                    if (fb.motor_id == 1) { p1 = fb.angle_deg; m_p1=1; }
                    if (fb.motor_id == 2) { y2 = fb.angle_deg; m_y2=1; }
                    if (fb.motor_id == 3) { y3 = fb.angle_deg; m_y3=1; }

//                    // [LOG]
//                    if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE){
//                        printf("[tx] recv fb id=%d angle=%.2fdeg\r\n", fb.motor_id, fb.angle_deg);
//                        xSemaphoreGive(printMutex);
//                    }
                }
                // 쌍 평균으로 합성: (0:-pitch, 1:+pitch), (2:+yaw, 3:-yaw)
                if (m_p0||m_p1) fb_pitch_deg = ( (-p0) + (p1) ) * 0.5f;
                if (m_y2||m_y3) fb_yaw_deg   = ( (y2)  + (-y3) ) * 0.5f;
            }

            /* 제어량 최종 송신 (시뮬레이션 진행중일 때만 송신) */
            if(sim_state == 0){
                // 큐에서 데이터를 받자마자 현재 시간을 ms 단위로 측정 (Tick 수 * Tick 당 ms)
                current_time_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

                // 송신 데이터에 모터 피드백 포함
                // 날개 제어량 수신처 : 언리얼 엔진, 모터
                // 날개 모터 피드백 수신처 : 언리얼 엔진
                data_len = snprintf(send_buffer, sizeof(send_buffer),
                                    "Pitch_Fin_Control_Amount: %.4f Yaw_Fin_Control_Amount: %.4f target_pitch: %.4f target_yaw: %.4f "
                                    "Pitch_Motor_Feedback: %.4f Yaw_Motor_Feedback: %.4f Send_Time: %lu",
                                    pitch_fin_deg_out, yaw_fin_deg_out, target_pitch, target_yaw,
                                    fb_pitch_deg, fb_yaw_deg,
                                    current_time_ms);
                // 언리얼에 UDP 송신
                sendto(udp_sock, send_buffer, data_len, 0, (struct sockaddr*)&client_addr, client_addr_len);

                // 언리얼에 송신한 문자열 출력(뮤텍스로 보호)
                if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
                    printf("[tx] send -> %s\r\n", send_buffer);
                    xSemaphoreGive(printMutex);
                }
            }
        }
    }
}
