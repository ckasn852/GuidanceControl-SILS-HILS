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
#include "../control/control_output_hw.h"

// 태스크 시간 측정
#include "xtime_l.h"
#include "../sys_stat/system_stats.h"   // [ADD] Tx(Build/Send) 통계 변수 사용

// 디버깅 뮤텍스, 소켓 뮤텍스
extern SemaphoreHandle_t printMutex;
extern SemaphoreHandle_t socketMutex;

// 모터 피드백 큐
#include "../hw_interface/hw_interface_task.h"

void tx_task(){
    // 송신 구간별 시간 측정을 위한 타이머
    XTime tSendStart,  tSendEnd;

    control_output sim_data;       // 언리얼로 보내는 가상 데이터
    real_sensor_data real_data;    // 하드웨어로 보내는 센서 데이터

    // 초기화
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

            // control_task 처리 후 언리얼 송신까지 소요 시간 계산
            // (Tx Send 시작 시각 - Control Start 시각)
            double delay_ctrl_to_tx = (double)(tSendStart - sim_data.ctrl_start_time) / (COUNTS_PER_SECOND / 1000000.0);
            if (delay_ctrl_to_tx < 0) delay_ctrl_to_tx = 0.0;


            //  데이터 포맷 변경
            // 1. Pitch/Yaw Fin (제어량)
            // 2. Target Pitch/Yaw (목표 각속도)
            // 3. Req 간격 / Req->Rx(RTT) / Rx->Ctrl / Ctrl->Tx (시간 데이터 4개)
            data_len = snprintf(send_buffer, sizeof(send_buffer),
                    "Pitch_Fin: %.4f Yaw_Fin: %.4f "
                    "Tgt_Pitch: %.4f Tgt_Yaw: %.4f "
                    "Req_Int: %.2f RTT: %.2f Rx2Ctrl: %.2f Ctrl2Tx: %.2f",
                    sim_data.pitch_fin_deg, sim_data.yaw_fin_deg,
                    sim_data.target_pitch, sim_data.target_yaw,
                    g_req_period_us,              // 1. req_task 송신 간격
                    g_last_rtt_us,                // 2. req->rx (RTT)
                    sim_data.rx_to_ctrl_delay_us, // 3. rx->control 소요 시간
                    (float)delay_ctrl_to_tx       // 4. control->tx 소요 시간
            );

            // 언리얼에 UDP 송신(소켓 뮤텍스 사용)
            if (xSemaphoreTake(socketMutex, portMAX_DELAY) == pdTRUE) {
                sendto(udp_sock, send_buffer, data_len, 0,
                       (struct sockaddr*)&client_addr, client_addr_len);
                xSemaphoreGive(socketMutex);
            }

            // 언리얼로 보낸 타임스탬프 출력(1초에 한 번씩)
			print_cnt++;
			if ((print_cnt % 50) == 0) {
				if (xSemaphoreTake(printMutex, 0) == pdTRUE) {
					printf("[TX] req_interval:%.1f req_recv_delta:%.1f rx_control_delay:%.1f control_to_unreal_delay:%.1f (us)\r\n",
						   g_req_period_us,
						   g_last_rtt_us,
						   sim_data.rx_to_ctrl_delay_us,
						   (float)delay_ctrl_to_tx
					);
					xSemaphoreGive(printMutex);
				}
			}

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
