#define YAW_PITCH_TOLERANCE 0.3f // 오차 X, Y가 ±0.3 이내면 정지
#include "control_task.h"
#include "control_queue.h"
#include "../ibvs/ibvs_calculate.h"
#include "../pid/pid.h"
#include <math.h>
#include <stdbool.h>
#include "semphr.h"
#include "xtime_l.h"
#include "../motor/motor_driver_task.h"
#include "../control/pid_tuning_mode.h"

// main.c 에 정의된 뮤텍스 참조
extern SemaphoreHandle_t printMutex;
volatile uint32_t g_isr_cnt = 0;

void control_task(){

    control_data rx_from_queue;
    control_output tx_to_queue;

    // PID 컨트롤러 인스턴스
    PID_t pid_yaw;
    PID_t pid_pitch;

    float velocity_out[6];                      // 목표 각속도 벡터
    float x, y, dist, cur_pitch, cur_yaw;      // 타겟 x,y 좌표(오차), 거리, 동체의 현재 Pitch, Yaw 각속도
    float target_pitch, target_yaw;            // Pitch, Yaw 방향 목표 각속도
    float pitch_fin_deg, yaw_fin_deg;          // Pitch, Yaw 방향 목표 날개 제어량
    float pitch_fin_deg_rx, yaw_fin_deg_rx;    // Pitch, Yaw 날개 현재 각도
    int sim_state = 0;

    // 동체 회전 피드 전달용
    float cur_mis_pitch = 0.0f;
    float cur_mis_yaw   = 0.0f;

    // PID Gain Pitch, Yaw 각각 적용
    pid_init(&pid_pitch, 30.0f, 15.0f, 5.0f, 40.0f, -40.0f);  // 20ms 최적 PID 설정
    pid_init(&pid_yaw,   30.0f, 15.0f, 5.0f, 40.0f, -40.0f);

    XTime time_last;
    XTime_GetTime(&time_last);

    control_data last_rx = {0};

    // 현재 튜닝 플래그(기본: 미적용=1)
    static int g_tuning_active = 1;

    // 데이터 미수신 카운터
    static int no_data_watchdog = 0;
    const int MAX_NO_DATA_TICKS = 5; // 20ms * 5 = 100ms (0.1초간 통신 끊기면 비상)

    while(1)
    {
    	// 하드타이머 ISR이 보낸 Notify를 기다림(20ms마다)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 언리얼 PID 튜닝 업데이트 수신
        PIDUpdate_t upd;
        while (xQueueReceive(xPidUpdateQueue, &upd, 0) == pdTRUE) {
            if (upd.has_params == 1 && upd.tuning_active == 0) {
                pid_pitch.Kp = upd.Kp; pid_pitch.Ki = upd.Ki; pid_pitch.Kd = upd.Kd;
                pid_yaw.Kp   = upd.Kp; pid_yaw.Ki   = upd.Ki; pid_yaw.Kd   = upd.Kd;
                pid_pitch.integral = 0.0f; pid_pitch.pre_e = 0.0f;
                pid_yaw.integral   = 0.0f; pid_yaw.pre_e   = 0.0f;
                if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
                    printf("[Control_task][MOD] Applied PID from Unreal: P=%.3f I=%.3f D=%.3f (tuning_active=%d)\r\n",
                           upd.Kp, upd.Ki, upd.Kd, upd.tuning_active);
                    xSemaphoreGive(printMutex);
                }
            }
            g_tuning_active = upd.tuning_active;
        }

        XTime time_now;
        XTime_GetTime(&time_now);
        uint64_t dt_count = (time_now - time_last);
        time_last = time_now;
        float dt_real = (float)dt_count / (float)COUNTS_PER_SECOND;
        if (dt_real < 0.010f) dt_real = 0.010f;
        if (dt_real > 0.030f) dt_real = 0.030f;

        // printf("dt_real : %.4f\r\n", dt_real);

        if (xQueueReceive(xControlQueue, &rx_from_queue, 0) == pdTRUE) {
            last_rx = rx_from_queue;	// 최신 데이터 갱신
            no_data_watchdog = 0;    	// 카운터 초기화
        }else {
            // Rx가 느려서 데이터 수신 실패할 경우
            no_data_watchdog++;      // 카운터 증가
        }

		//  Failsafe -> 0.1초 이상 데이터가 안 오는 경우 비행 방향을 유지
		if (no_data_watchdog > MAX_NO_DATA_TICKS) {
			x = 0; y = 0;
		} else {
			// 정상 제어 (Rx가 조금 늦어도 last_rx 값으로 부드럽게 제어)
	        x = last_rx.img_x;
	        y = last_rx.img_y;
	        dist = last_rx.distance;
	        cur_yaw = last_rx.yaw_rate;
	        cur_pitch = last_rx.pitch_rate;
	        pitch_fin_deg_rx = last_rx.pitch_wing_deq;
	        yaw_fin_deg_rx = last_rx.yaw_wing_deq;
	        sim_state = last_rx.sim_state;
		}

        // 동체 회전 데이터, rx_task에서 p,y 값 파싱
        cur_mis_pitch = last_rx.p;
        cur_mis_yaw   = last_rx.y;

        if(sim_state == 0){
            pid_pitch.integral = 0;
            pid_yaw.integral = 0;
        }

        // ibvs 연산
        ibvs_calculate(x, y, dist, velocity_out);

        target_pitch = velocity_out[3];
        target_yaw   = velocity_out[4];

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
            cmd.motor_id = 2; cmd.angle =  yaw_fin_deg;  xQueueSend(xAngleQueue, &cmd, 0);
            cmd.motor_id = 3; cmd.angle = -yaw_fin_deg;  xQueueSend(xAngleQueue, &cmd, 0);

            // 모터 4-5 (동체)
            cmd.motor_id = 4; cmd.angle = -cur_mis_pitch; xQueueSend(xAngleQueue, &cmd, 0);
            cmd.motor_id = 5; cmd.angle =  cur_mis_yaw;   xQueueSend(xAngleQueue, &cmd, 0);

//            // 송신 로그
//            if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
//                printf("[control] pitch_cmd=%.2f yaw_cmd=%.2f (sent to motors)\r\n",
//                       pitch_fin_deg, yaw_fin_deg);
//                xSemaphoreGive(printMutex);
//            }
        }

        // 최종 날개 제어각, 시뮬레이션 수행 상태를 큐에 넣음
        tx_to_queue.pitch_fin_deg = pitch_fin_deg;
        tx_to_queue.yaw_fin_deg	= yaw_fin_deg;
        tx_to_queue.target_pitch = target_pitch;
        tx_to_queue.target_yaw   = target_yaw;
        tx_to_queue.sim_state = sim_state;

        // 큐로 제어 데이터 push
        xQueueOverwrite(xControlOutQueue, &tx_to_queue);
    }
}
