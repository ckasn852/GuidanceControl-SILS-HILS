#ifndef SYSTEM_STATS_H_
#define SYSTEM_STATS_H_

#include <stdint.h>
#include "xtime_l.h"

/* 기존 전역 변수들 */
extern float    g_time_req_us;
extern float    g_time_rx_us;
extern float    g_time_ctrl_us;
extern float    g_time_hw_us;
extern float    g_time_tx_us;

extern double   g_sum_req_us;
extern double   g_sum_rx_us;
extern double   g_sum_ctrl_us;
extern double   g_sum_hw_us;
extern double   g_sum_tx_us;

extern double   g_avg_req_us;
extern double   g_avg_rx_us;
extern double   g_avg_ctrl_us;
extern double   g_avg_hw_us;
extern double   g_avg_tx_us;

extern double   g_wcet_req_us;
extern double   g_wcet_rx_us;
extern double   g_wcet_ctrl_us;
extern double   g_wcet_hw_us;
extern double   g_wcet_tx_us;

extern uint32_t g_cnt_req;
extern uint32_t g_cnt_rx;
extern uint32_t g_cnt_ctrl;
extern uint32_t g_cnt_hw;
extern uint32_t g_cnt_tx;

extern float    g_time_tx_build_us;   // 한 번의 build 수행 시간
extern float    g_time_tx_send_us;    // 한 번의 send 수행 시간

extern double   g_sum_tx_build_us;
extern double   g_sum_tx_send_us;

extern double   g_avg_tx_build_us;
extern double   g_avg_tx_send_us;

extern double   g_wcet_tx_build_us;
extern double   g_wcet_tx_send_us;

extern uint32_t g_cnt_tx_build;
extern uint32_t g_cnt_tx_send;

// 송수신 간격 측정 타임스탬프
extern XTime g_req_send_timestamp;
extern volatile int g_req_pending;
extern float g_last_rtt_us;
extern float g_req_period_us;

#endif /* SYSTEM_STATS_H_ */
