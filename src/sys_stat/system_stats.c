#include "system_stats.h"

/* 전역 변수 실체 정의 */
/* 초기화는 여기서 한 번만 수행 */

volatile float g_time_rx_us   = 0.0f;
volatile float g_time_ctrl_us = 0.0f;
volatile float g_time_hw_us   = 0.0f;
volatile float g_time_tx_us   = 0.0f;

volatile double g_wcet_control_us = 0.0;
