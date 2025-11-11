#define YAW_PITCH_TOLERANCE 0.3f // 오차 X, Y가 ±0.3 이내면 정지
#include "control_task.h"
#include "control_queue.h"
#include "../ibvs/ibvs_calculate.h"
#include "../pid/pid.h"
#include <math.h>
#include <stdbool.h>
#include "semphr.h"
#include "xtime_l.h"
#include "FreeRTOS.h"
#include "pid_tuning_mode.h"

// main.c 에 정의된 뮤텍스 참조
extern SemaphoreHandle_t printMutex;

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

    // PID Gain Pitch, Yaw 각각 적용
//  pid_init(&pid_pitch, 50.0f, 20.0f, 0.0001f, 200.0f, -200.0f);  // 50ms 최적 PID 설정
//  pid_init(&pid_yaw,   50.0f, 20.0f, 0.0001f, 200.0f, -200.0f);
    pid_init(&pid_pitch, 30.0f, 15.0f, 5.0f, 80.0f, -80.0f);  // 30ms 최적 PID 설정
    pid_init(&pid_yaw,   30.0f, 15.0f, 5.0f, 80.0f, -80.0f);

    // ZYNQ 보드 RTOS 20ms 고정 주기 설정
    const TickType_t period = pdMS_TO_TICKS(20);
    TickType_t last = xTaskGetTickCount();

    // dt_real 계산용 고해상도 타임스탬프 초기화
    XTime time_last;
    XTime_GetTime(&time_last);

    // 최신 샘플 유지를 위해 이전 스텝 데이터 변수 선언
    control_data last_rx = {0};

    // 현재 튜닝 플래그(기본: 미적용=1)
    static int g_tuning_active = 1;

    while(1)
    {
        vTaskDelayUntil(&last, period); // 20ms 주기

        // 언리얼로부터 PID 업데이트/Control 플래그 논블로킹 수신
        PIDUpdate_t upd;
        while (xQueueReceive(xPidUpdateQueue, &upd, 0) == pdTRUE) {
            if (upd.has_params == 1 && upd.tuning_active == 0) {					// 튜닝 적용 중일 때만 반영
                pid_pitch.Kp = upd.Kp; pid_pitch.Ki = upd.Ki; pid_pitch.Kd = upd.Kd;
                pid_yaw.Kp   = upd.Kp; pid_yaw.Ki   = upd.Ki; pid_yaw.Kd   = upd.Kd;
                pid_pitch.integral = 0.0f; pid_pitch.pre_e = 0.0f;
                pid_yaw.integral   = 0.0f; pid_yaw.pre_e   = 0.0f;
//                if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
//                    printf("[Control_task][MOD] Applied PID from Unreal: P=%.3f I=%.3f D=%.3f (tuning_active=%d)\r\n",
//                           upd.Kp, upd.Ki, upd.Kd, upd.tuning_active);
//                    xSemaphoreGive(printMutex);
//                }
            }
            g_tuning_active = upd.tuning_active;                         // 0/1 상태 갱신
        }

        // 실제 경과 시간 dt_real 계산
        XTime time_now;
        XTime_GetTime(&time_now);
        uint64_t dt_count = (time_now - time_last);
        time_last = time_now;
        float dt_real = (float)dt_count / (float)COUNTS_PER_SECOND;
        if (dt_real < 0.010f) dt_real = 0.010f;    // dt 클램핑 하한(10ms)
        if (dt_real > 0.030f) dt_real = 0.030f;    // dt 클램핑 상한(30ms)

        // printf("dt_real : %.4f\r\n", dt_real);

        // 받은 데이터 있으면 갱신, 없으면 직전값 유지
        if (xQueueReceive(xControlQueue, &rx_from_queue, 0) == pdTRUE) {
            last_rx = rx_from_queue;
        }

        // 수신 데이터를 지역 변수에 할당 (last_rx 기준)
        x = last_rx.img_x;
        y = last_rx.img_y;
        dist = last_rx.distance;
        cur_yaw = last_rx.yaw_rate;
        cur_pitch = last_rx.pitch_rate;
        pitch_fin_deg_rx = last_rx.pitch_wing_deq;
        yaw_fin_deg_rx = last_rx.yaw_wing_deq;
        sim_state = last_rx.sim_state;

//        // rx_task로 부터 받은 데이터를 출력(뮤텍스로 보호)
//        if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
//        	printf("[Control_task]x: %.4f y: %.4f dist: %.4f pitch: %.4f yaw: %.4f, PW:%.4f, YW:%.4f, sim_state: %d \r\n\n",
//        			x, y, dist, cur_pitch, cur_yaw, pitch_fin_deg_rx, yaw_fin_deg_rx, sim_state);
//        	xSemaphoreGive(printMutex);
//        }

        // 시뮬레이션 종료 시 누적오차 초기화
        if(sim_state == 0){
            pid_pitch.integral = 0;
            pid_yaw.integral = 0;
        }

        // printf("pid_pitch.integral: %.4f, pid_yaw.integral: %.4f\r\n", pid_pitch.integral, pid_yaw.integral);

        // IBVS 연산 함수 호출 -> Pitch, Yaw 방향 목표 각속도를 velocity_out 배열에 저장
        ibvs_calculate(x, y, dist, velocity_out);

        // Pitch 방향 목표 각속도: velocity_out[3], Yaw 방향 목표 각속도: velocity_out[4]
        target_pitch = velocity_out[3];
        target_yaw   = velocity_out[4];

        // 데드존 적용 : 오차가 허용 범위 내면, 목표 각속도 명령을 0으로 설정하여 움직임 정지
        if (fabsf(x) < YAW_PITCH_TOLERANCE) {
            target_yaw = 0.0f;
            pid_yaw.integral = 0; // 적분 안티 와인딩 : 목표 도달 이 후 미세 제어 시 오버슈트방지를 위해 누적오차 초기화
        }

        if (fabsf(y) < YAW_PITCH_TOLERANCE) {
            target_pitch = 0.0f;
            pid_pitch.integral = 0; // 목표 도달 이 후 미세 제어 시 오버슈트방지를 위해 누적오차 초기화
        }

//      // 목표 각속도 출력
//      if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
//          printf("[Control_task]Target pitch: %.4f, Target yaw: %.4f\r\n", velocity_out[3], velocity_out[4]);
//          xSemaphoreGive(printMutex);
//      }

        // 동체 각속도 에러(목표 각속도 - 현재 각속도) PID 입력 후 날개 각도 제어량 출력
        if(target_pitch != 0){
            // printf("[Pitch PID]\n");
            pitch_fin_deg = pid_calculation(&pid_pitch, target_pitch, cur_pitch, dt_real);		// pitch 방향 날개 각도 제어량
        }
        else{
            pitch_fin_deg = 0;
        }
        if(target_yaw != 0){
            // printf("[Yaw PID]\n");
            yaw_fin_deg = pid_calculation(&pid_yaw, target_yaw, cur_yaw, dt_real);				// yaw 방향 날개 각도 제어량
        }
        else{
            yaw_fin_deg = 0;
        }

        // 최종 날개 제어각을 큐에 넣음
        tx_to_queue.pitch_fin_deg = pitch_fin_deg;
        tx_to_queue.yaw_fin_deg	= yaw_fin_deg;
        tx_to_queue.sim_state = sim_state;

        // 큐로 제어 데이터 push (큐 길이 1)
        xQueueSend(xControlOutQueue, &tx_to_queue, 0);
        // xQueueOverwrite(xControlOutQueue, &tx_to_queue);
    }
}
