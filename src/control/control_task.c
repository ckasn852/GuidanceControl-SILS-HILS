#define YAW_PITCH_TOLERANCE 0.3f // 오차 X, Y가 ±0.3 이내면 정지
#include "control_task.h"
#include "control_queue.h"
#include "../ibvs/ibvs_calculate.h"
#include "../pid/pid.h"
#include <math.h>
#include <stdbool.h>
#include "semphr.h"
#include "xtime_l.h"
#include "../hw_interface/hw_interface_task.h"
#include "../control/pid_tuning_mode.h"
#include "../control/control_output.h"

// 태스크 시간 측정
#include "xtime_l.h"
#include "../sys_stat/system_stats.h"
#include "task.h"                               // xTaskGetTickCount 사용을 위해 추가

// main.c 에 정의된 뮤텍스 참조
extern SemaphoreHandle_t printMutex;

//  ISR에서 증가시키는 전역 카운터는 main.c에만 정의, 여기서는 extern으로 사용
extern volatile uint32_t g_isr_cnt;

// Control Task는 타이머 Notify만 사용하므로 무한 대기 (타임아웃 제거)
const TickType_t CHAIN_TIMEOUT = portMAX_DELAY;

void control_task(){

    control_data rx_from_queue;
    control_output tx_to_queue;

    // PID 컨트롤러 인스턴스
    PID_t pid_yaw;
    PID_t pid_pitch;

    float velocity_out[6];                              // 목표 각속도 벡터
    float x, y, dist, cur_pitch, cur_yaw;               // 타겟 x,y 좌표(오차), 거리, 동체의 현재 Pitch, Yaw 각속도
    float target_pitch, target_yaw;                     // Pitch, Yaw 방향 목표 각속도
    float pitch_fin_deg, yaw_fin_deg;                   // Pitch, Yaw 방향 목표 날개 제어량
    float pitch_fin_deg_rx, yaw_fin_deg_rx;             // Pitch, Yaw 날개 현재 각도
    int sim_state = 0;

    // 동체 회전 피드 전달용
    float cur_mis_pitch = 0.0f;
    float cur_mis_yaw   = 0.0f;

    // PID Gain Pitch, Yaw 각각 적용
//    pid_init(&pid_pitch, 30.0f, 15.0f, 5.0f, 40.0f, -40.0f);  // 20ms 최적 PID 설정
//    pid_init(&pid_yaw,   30.0f, 15.0f, 5.0f, 40.0f, -40.0f);
    pid_init(&pid_pitch, 100.0f, 0.0f, 0.1f, 40.0f, -40.0f);
    pid_init(&pid_yaw,   100.0f, 0.0f, 0.1f, 40.0f, -40.0f);

    XTime time_last;
    XTime_GetTime(&time_last);

    // 태스크 수행시간 측정용 변수
    XTime tStart, tEnd;

    control_data last_rx = {0};

    // 현재 튜닝 플래그(기본: 미적용=1)
    static int g_tuning_active = 1;

    // 데이터 미수신 카운터
    static int no_data_watchdog = 0;
    const int MAX_NO_DATA_TICKS = 5; // 20ms * 5 = 100ms (0.1초간 통신 끊기면 비상)

    // 주기 확인용 Tick 기록 변수
    static TickType_t lastTick = 0;
    static uint32_t   loopCnt  = 0;
    // 집계 로그 1회 출력 여부 플래그
    static BaseType_t statsPrinted = pdFALSE;

    while(1)
    {
        /* 타이머 ISR(10ms/100Hz → 20ms마다 Control 깨움)의 Notify를 대기 */
        ulTaskNotifyTake(pdTRUE, CHAIN_TIMEOUT);

        // control_task 기상 간격(주기) 측정용 로그
        {
            TickType_t nowTick = xTaskGetTickCount();
            if (lastTick != 0) {
                TickType_t diffTick = nowTick - lastTick;
                uint32_t diffMs = diffTick * portTICK_PERIOD_MS;
                loopCnt++;
            }
            lastTick = nowTick;
        }

        // 언리얼 PID 튜닝 업데이트 수신
        PIDUpdate_t upd;
        while (xQueueReceive(xPidUpdateQueue, &upd, 0) == pdTRUE) {
            if (upd.has_params == 1 && upd.tuning_active == 0) {
                pid_pitch.Kp = upd.Kp; pid_pitch.Ki = upd.Ki; pid_pitch.Kd = upd.Kd;
                pid_yaw.Kp   = upd.Kp; pid_yaw.Ki   = upd.Ki; pid_yaw.Kd   = upd.Kd;
                pid_pitch.integral = 0.0f; pid_pitch.pre_e = 0.0f;
                pid_yaw.integral   = 0.0f; pid_yaw.pre_e   = 0.0f;
                if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
                    printf("[Control_task] Applied PID from Unreal\r\n");
                    xSemaphoreGive(printMutex);
                }
            }
            g_tuning_active = upd.tuning_active;
        }

        // dt 측정용 변수
        XTime time_now;
        XTime_GetTime(&time_now);
        uint64_t dt_count = (time_now - time_last);
        time_last = time_now;
        float dt_real = (float)dt_count / (float)COUNTS_PER_SECOND;
        if (dt_real < 0.010f) dt_real = 0.010f;
        if (dt_real > 0.030f) dt_real = 0.030f;

        /* 데이터 수신 로직: 이번 주기에 새 데이터가 있으면 즉시 사용, 없으면 last_rx 유지 */
        BaseType_t rx_status = xQueueReceive(xControlQueue, &rx_from_queue, 0);

        /* WCET 측정 시작 (순수 연산 시작점) */
        XTime_GetTime(&tStart);

        //  Rx -> Control 데이터 지연 시간 계산
        // req 송신 시각 + RTT = rx 수신 완료 시각(추정)
        // 현재(Control 시작) - rx 수신 완료 시각 = 지연 시간
        double rtt_counts = (double)g_last_rtt_us * (COUNTS_PER_SECOND / 1000000.0);
        XTime tRxFinishEstimated = g_req_send_timestamp + (XTime)rtt_counts;
        double rx_to_ctrl_delay = (double)(tStart - tRxFinishEstimated) / (COUNTS_PER_SECOND / 1000000.0);

        // 음수 방지 (타이밍 오차 등으로 인한)
        if (rx_to_ctrl_delay < 0.0) rx_to_ctrl_delay = 0.0;

        if (rx_status == pdTRUE) {
            last_rx = rx_from_queue;    // 최신 데이터 갱신
            no_data_watchdog = 0;       // 카운터 초기화
        } else {
            // 이번 주기에 새 데이터 없음
            no_data_watchdog++;         // 카운터 증가
        }

        //  Failsafe -> 0.1초 이상 데이터가 안 오는 경우 비행 방향을 유지
        if (no_data_watchdog > MAX_NO_DATA_TICKS) {
            x = 0; y = 0;
        } else {
            // 정상 제어
            x = last_rx.img_x;
            y = last_rx.img_y;
            dist = last_rx.distance;
            cur_yaw = last_rx.yaw_rate;
            cur_pitch = last_rx.pitch_rate;
            pitch_fin_deg_rx = last_rx.pitch_wing_deq;
            yaw_fin_deg_rx = last_rx.yaw_wing_deq;
            sim_state = last_rx.sim_state;
            cur_mis_pitch = last_rx.p;
            cur_mis_yaw   = last_rx.y;
        }

        if(sim_state == 0){
            pid_pitch.integral = 0;
            pid_yaw.integral = 0;
        }

        // ibvs 연산
        ibvs_calculate(x, y, dist, velocity_out);
        target_pitch = velocity_out[3];
        target_yaw   = velocity_out[4];

        // 데드존 적용
        if (fabsf(x) < YAW_PITCH_TOLERANCE) {
            target_yaw = 0.0f;
            pid_yaw.integral = 0;
        }
        if (fabsf(y) < YAW_PITCH_TOLERANCE) {
            target_pitch = 0.0f;
            pid_pitch.integral = 0;
        }

        if(target_pitch != 0){
            pitch_fin_deg = pid_calculation(&pid_pitch, target_pitch, cur_pitch, dt_real);
        } else {
            pitch_fin_deg = 0;
        }
        if(target_yaw != 0){
            yaw_fin_deg = pid_calculation(&pid_yaw, target_yaw, cur_yaw, dt_real);
        } else {
            yaw_fin_deg = 0;
        }

        // 모터로 명령 송신
        if (xAngleQueue) {
            ServoCommand_t cmd;

            // 모터 0-3 (날개)
            cmd.motor_id = 0; cmd.angle = -pitch_fin_deg; xQueueSend(xAngleQueue, &cmd, 0);
            cmd.motor_id = 1; cmd.angle =  pitch_fin_deg; xQueueSend(xAngleQueue, &cmd, 0);
            cmd.motor_id = 2; cmd.angle =  yaw_fin_deg;   xQueueSend(xAngleQueue, &cmd, 0);
            cmd.motor_id = 3; cmd.angle = -yaw_fin_deg;   xQueueSend(xAngleQueue, &cmd, 0);

            // 모터 4-5 (동체)
            cmd.motor_id = 4; cmd.angle = -cur_mis_pitch; xQueueSend(xAngleQueue, &cmd, 0);
            cmd.motor_id = 5; cmd.angle =  cur_mis_yaw;   xQueueSend(xAngleQueue, &cmd, 0);
        }

        // 최종 날개 제어각, 시뮬레이션 수행 상태를 큐에 넣음
        tx_to_queue.pitch_fin_deg = pitch_fin_deg;
        tx_to_queue.yaw_fin_deg   = yaw_fin_deg;
        tx_to_queue.target_pitch  = target_pitch;
        tx_to_queue.target_yaw    = target_yaw;
        tx_to_queue.sim_state     = sim_state;

        // 시간 측정 데이터 패킹
        tx_to_queue.rx_to_ctrl_delay_us = (float)rx_to_ctrl_delay; // 3. Rx->Ctrl 지연
        tx_to_queue.ctrl_start_time = tStart;                      // 4. Ctrl 시작 시간 (Tx에서 사용)

        // 큐로 제어 데이터 push
        xQueueOverwrite(xControlOutQueue, &tx_to_queue);

        /* WCET / 평균 시간 측정 종료 및 기록 */
        XTime_GetTime(&tEnd);
        double current_exec_us = (double)(tEnd - tStart) / (COUNTS_PER_SECOND / 1000000.0);

        g_time_ctrl_us = (float)current_exec_us;
        g_sum_ctrl_us  += current_exec_us;
        g_cnt_ctrl++;
        g_avg_ctrl_us = (g_cnt_ctrl > 0) ? (g_sum_ctrl_us / g_cnt_ctrl) : 0.0;

        if (current_exec_us > g_wcet_ctrl_us) {
            g_wcet_ctrl_us = current_exec_us;
        }
    }
}
