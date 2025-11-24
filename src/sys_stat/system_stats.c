#include "system_stats.h"
#include <stdio.h>

/* =========================================
 * 전역 변수 실체 정의 (메모리 할당)
 * ========================================= */

/* 1. 수행 시간 측정 변수들 */
float    g_time_req_us    = 0.0f;
float    g_time_rx_us     = 0.0f;
float    g_time_ctrl_us   = 0.0f;
float    g_time_hw_us     = 0.0f;
float    g_time_tx_us     = 0.0f;

double   g_sum_req_us     = 0.0;
double   g_sum_rx_us      = 0.0;
double   g_sum_ctrl_us    = 0.0;
double   g_sum_hw_us      = 0.0;
double   g_sum_tx_us      = 0.0;

double   g_avg_req_us     = 0.0;
double   g_avg_rx_us      = 0.0;
double   g_avg_ctrl_us    = 0.0;
double   g_avg_hw_us      = 0.0;
double   g_avg_tx_us      = 0.0;

double   g_wcet_req_us    = 0.0;
double   g_wcet_rx_us     = 0.0;
double   g_wcet_ctrl_us   = 0.0;
double   g_wcet_hw_us     = 0.0;
double   g_wcet_tx_us     = 0.0;

uint32_t g_cnt_req        = 0;
uint32_t g_cnt_rx         = 0;
uint32_t g_cnt_ctrl       = 0;
uint32_t g_cnt_hw         = 0;
uint32_t g_cnt_tx         = 0;

/* 2. Tx Build / Send 분리 통계 변수 */
float    g_time_tx_build_us  = 0.0f;
float    g_time_tx_send_us   = 0.0f;

double   g_sum_tx_build_us   = 0.0;
double   g_sum_tx_send_us    = 0.0;

double   g_avg_tx_build_us   = 0.0;
double   g_avg_tx_send_us    = 0.0;

double   g_wcet_tx_build_us  = 0.0;
double   g_wcet_tx_send_us   = 0.0;

uint32_t g_cnt_tx_build      = 0;
uint32_t g_cnt_tx_send       = 0;

/* 3. 송수신 간격 및 지연 측정용 변수 */
XTime g_req_send_timestamp = 0;
volatile int g_req_pending = 0;
float g_last_rtt_us = 0.0f;
float g_req_period_us = 0.0f;
