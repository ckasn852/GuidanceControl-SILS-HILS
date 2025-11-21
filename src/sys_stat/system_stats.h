/* system_stats.h O(태스크 시간 측정용) */
#ifndef SYSTEM_STATS_H
#define SYSTEM_STATS_H

// 각 태스크의 최신 실행 시간 (단위: us)
// volatile을 사용하여 최적화를 방지하고 항상 메모리 값을 참조하도록 함
extern volatile float g_time_rx_us;
extern volatile float g_time_ctrl_us;
extern volatile float g_time_hw_us;
extern volatile float g_time_tx_us;

// Control Task의 WCET (최악 실행 시간) 기록용
extern volatile double g_wcet_control_us;

#endif
