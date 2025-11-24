// tx_task.c

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
#include "../control/control_output_hw.h" // [MOD] 헤더 복구

// 태스크 시간 측정
#include "xtime_l.h"
#include "../sys_stat/system_stats.h"   // Tx(Build/Send) 통계 변수 사용

// 디버깅 뮤텍스, 소켓 뮤텍스
extern SemaphoreHandle_t printMutex;
extern SemaphoreHandle_t socketMutex;

// 모터 피드백 큐
// #include "../hw_interface/hw_interface_task.h" // (기존 주석 유지)
extern QueueHandle_t xRealDataQueue; // 큐 핸들 extern

void tx_task(){
    // 송신 구간별 시간 측정을 위한 타이머
    XTime tSendStart,  tSendEnd;

    control_output sim_data;       // 언리얼로 보내는 가상 데이터
    real_sensor_data real_data;    // 하드웨어로 보내는 센서 데이터

    // 초기화
    // [MOD] 초기화 복구
    memset(&real_data, 0, sizeof(real_data));

    int data_len = 0;
    char send_buffer[TX_BUFFER_LEN];

    // UDP 송신 타임아웃(주기 침범 방지, 2ms)
    struct timeval stv = { .tv_sec = 0, .tv_usec = 2000 };
    setsockopt(udp_sock, SOL_SOCKET, SO_SNDTIMEO, &stv, sizeof(stv));

    // 로그 출력 주기 제어용 카운터
    static uint32_t print_cnt = 0;

    while(1)
    {
        // Control 태스크 대기
        if (xQueueReceive(xControlOutQueue, &sim_data, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* 제어량 최종 송신 (시뮬레이션 진행중일 때만 송신) */
        if(sim_data.sim_state == 0){

            /* =======================
             * 2) sendto 송신 시간 측정 (여기서 데이터를 보내는 시점으로 간주)
             * ======================= */
            XTime_GetTime(&tSendStart);

            // IMU 데이터 수신 (Non-blocking)
            if (xRealDataQueue != NULL) {
                xQueueReceive(xRealDataQueue, &real_data, 0);
            }

            // Control Task가 처리 완료(ctrl_finish_time)한 시점부터
            // Tx Task에서 sendto를 호출하기 전까지의 지연 시간 계산 // [MOD]
            double delay_ctrl_to_tx = 0.0;                                     // [MOD]
            if (sim_data.ctrl_finish_time != 0 &&                              // [MOD]
                tSendStart > sim_data.ctrl_finish_time) {                      // [MOD]
                delay_ctrl_to_tx =
                    (double)(tSendStart - sim_data.ctrl_finish_time)          // [MOD]
                    / (COUNTS_PER_SECOND / 1000000.0);                         // [MOD]
            }
            if (delay_ctrl_to_tx < 0) delay_ctrl_to_tx = 0.0;                  // [MOD]

            //  데이터 포맷 변경
            // 1. Pitch/Yaw Fin (제어량)
            // 2. Target Pitch/Yaw (목표 각속도)
            // 3. Req 간격 / Req->Rx(RTT) / Control 처리 시간 / Control 종료~Tx 송신까지 지연
            // 4. IMU Pitch/Yaw 추가
            data_len = snprintf(send_buffer, sizeof(send_buffer),
                    "Pitch_Fin: %.4f Yaw_Fin: %.4f "
                    "Tgt_Pitch: %.4f Tgt_Yaw: %.4f "
                    "Req_Int: %.2f RTT: %.2f Ctrl_Proc: %.2f Ctrl2Tx: %.2f " // [MOD]
                    "IMU_Pitch: %.2f IMU_Yaw: %.2f",                          // [MOD]
                    sim_data.pitch_fin_deg, sim_data.yaw_fin_deg,
                    sim_data.target_pitch, sim_data.target_yaw,
                    sim_data.req_period_us,            // 1. req_task 송신 간격 (Queue)
                    sim_data.rtt_us,                   // 2. req->rx (Queue)
                    sim_data.ctrl_proc_us,             // 3. Control 처리 시간
                    (float)delay_ctrl_to_tx,           // 4. Control 종료~Tx 송신까지 지연
                    real_data.real_imu_pitch,          // 5. IMU Pitch
                    real_data.real_imu_yaw             // 6. IMU Yaw
            );

            // 언리얼에 UDP 송신(소켓 뮤텍스 사용)
            if (xSemaphoreTake(socketMutex, portMAX_DELAY) == pdTRUE) {
                sendto(udp_sock, send_buffer, data_len, 0,
                       (struct sockaddr*)&client_addr, client_addr_len);
                xSemaphoreGive(socketMutex);
            }

            // 디버그 출력
            printf("[TX] req_rtt:%.3f ctrl_proc:%.3f ctrl2tx:%.3f (us)\r\n",   // [MOD]
                   sim_data.rtt_us,                                            // [MOD]
                   sim_data.ctrl_proc_us,                                      // [MOD]
                   (float)delay_ctrl_to_tx                                     // [MOD]
            );

            XTime_GetTime(&tSendEnd);

            double send_us =
                (double)(tSendEnd - tSendStart) / (COUNTS_PER_SECOND / 1000000.0);

            // Tx(Send) 통계 업데이트
            g_time_tx_send_us = (float)send_us;
            g_sum_tx_send_us  += send_us;
            g_cnt_tx_send++;
            g_avg_tx_send_us = (g_cnt_tx_send > 0)
                               ? (g_sum_tx_send_us / g_cnt_tx_send)
                               : 0.0;
            if (send_us > g_wcet_tx_send_us) {
                g_wcet_tx_send_us = send_us;
            }
        }
    }
}
