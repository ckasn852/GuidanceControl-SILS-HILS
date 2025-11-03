
#define YAW_PITCH_TOLERANCE 0.3f // 오차 X, Y가 ±0.3 이내면 정지
#include "control_task.h"
#include "control_queue.h"
#include "../ibvs/ibvs_calculate.h"
#include "../pid/pid.h"
#include <math.h>
#include <stdbool.h>
#include "semphr.h"
#include "../motor/motor_driver_task.h"
// main.c 에 정의된 뮤텍스 참조
extern SemaphoreHandle_t printMutex;

void control_task(){


	control_data rx_from_queue;
	control_output tx_to_queue;

	// PID 컨트롤러 인스턴스
	PID_t pid_yaw;
	PID_t pid_pitch;

	float velocity_out[6]; 						// 목표 각속도 벡터
	float x, y, dist, cur_pitch, cur_yaw, cur_mis_pitch, cur_mis_yaw;		// 타겟 x,y 좌표(오차), 거리, 동체의 현재 Pitch, Yaw 각속도, 미사일 pitch, 미사일 yaw
	float target_pitch, target_yaw;				// Pitch, Yaw 방향 목표 각속도
	float pitch_fin_deg, yaw_fin_deg; 			// Pitch, Yaw 방향 목표 날개 제어량
	float pitch_fin_deg_rx, yaw_fin_deg_rx; 	// Pitch, Yaw 날개 현재 각도
	bool sim_state = false;

	// PID Gain Pitch, Yaw 각각 적용
//	pid_init(&pid_pitch, 50.0f, 20.0f, 0.0001f, 200.0f, -200.0f);  // 50ms 최적 PID 설정
//  pid_init(&pid_yaw, 50.0f, 20.0f, 0.0001f, 200.0f, -200.0f);
    pid_init(&pid_pitch, 160.0f, 0.00001f, 2.5f, 80.0f, -80.0f);  // 30ms 최적 PID 설정
    pid_init(&pid_yaw, 160.0f, 0.00001f, 2.5f, 80.0f, -80.0f);

	const TickType_t xFrequency = pdMS_TO_TICKS(30); // 30ms 주기 (PID 설정과 일치)
    TickType_t xLastWakeTime = xTaskGetTickCount(); // 현재 시간 저장

    ServoCommand_t hils_cmd;




	while(1)
	    {
			// 30ms 주기로 실행
			vTaskDelayUntil(&xLastWakeTime, xFrequency);

			// 데이터가 올 때까지 대기
			if (xQueueReceive(xControlQueue, &rx_from_queue, pdMS_TO_TICKS(0)) == pdTRUE)
			{
				// 수신 데이터를 지역 변수에 할당
				x = rx_from_queue.img_x,
				y = rx_from_queue.img_y;
				dist = rx_from_queue.distance;
				cur_yaw = rx_from_queue.yaw_rate;
				cur_pitch = rx_from_queue.pitch_rate;
				cur_mis_pitch = rx_from_queue.pitch;
				cur_mis_yaw = rx_from_queue.yaw;
				pitch_fin_deg_rx = rx_from_queue.pitch_wing_deq;
				yaw_fin_deg_rx = rx_from_queue.yaw_wing_deq;
				sim_state = rx_from_queue.sim_state;

				// rx_task로 부터 받은 데이터를 출력(뮤텍스로 보호)
				if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
					printf("[Control_task]x: %.4f y: %.4f dist: %.4f pitch rate: %.4f yaw rate: %.4f, pitch: %.4f, yaw: %.4f, PW:%.4f, YW:%.4f, sim_state: %d \r\n\n",
							x, y, dist, cur_pitch, cur_yaw, cur_mis_pitch, cur_mis_yaw, pitch_fin_deg_rx, yaw_fin_deg_rx, sim_state);
					xSemaphoreGive(printMutex);
				}

				// 시뮬레이션 종료 시 누적오차 초기화
				if(sim_state == 0){
					pid_pitch.integral = 0;
					pid_yaw.integral = 0;
				}

				// IBVS 계산
				ibvs_calculate(x, y, dist, velocity_out);

				//  Pitch 각속도 목표 : velocity_out[3], Yaw 각속도 목표: velocity_out[4]
				target_pitch = velocity_out[3];
				target_yaw = velocity_out[4];

				// 데드존 적용 : 오차가 허용 범위 내면, 목표 각속도 명령을 0으로 설정하여 움직임 정지
				if (fabsf(x) < YAW_PITCH_TOLERANCE) {
					target_yaw = 0.0f;
					pid_yaw.integral = 0; // 적분 안티 와인딩 : 목표 도달 이 후 미세 제어 시 오버슈트방지를 위해 누적오차 초기화
				}

				if (fabsf(y) < YAW_PITCH_TOLERANCE) {
					target_pitch = 0.0f;
					pid_pitch.integral = 0; // 목표 도달 이 후 미세 제어 시 오버슈트방지를 위해 누적오차 초기화
				}

				// 목표 각속도 출력
				if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
					printf("[Control_task]Target pitch: %.4f, Target yaw: %.4f\r\n", velocity_out[3], velocity_out[4]);
					xSemaphoreGive(printMutex);
				}

				// PID 연산 후 Yaw, Pitch 날개 각도 제어 명령 출력
				if(target_pitch != 0){
					// printf("[Pitch PID]\n");
					pitch_fin_deg = pid_calculation(&pid_pitch, target_pitch, cur_pitch);
				}
				else{
					pitch_fin_deg = 0;
				}
				if(target_yaw != 0){
					// printf("[Yaw PID]\n");
					yaw_fin_deg = pid_calculation(&pid_yaw, target_yaw, cur_yaw);
				}
				else{
					yaw_fin_deg = 0;
				}

				// 모터 전송 //


				hils_cmd.motor_id = 0; hils_cmd.angle = -pitch_fin_deg; xQueueSend(xAngleQueue, &hils_cmd, (TickType_t)0);

				hils_cmd.motor_id = 1; hils_cmd.angle = pitch_fin_deg;  xQueueSend(xAngleQueue, &hils_cmd, (TickType_t)0);

				hils_cmd.motor_id = 2; hils_cmd.angle = yaw_fin_deg;    xQueueSend(xAngleQueue, &hils_cmd, (TickType_t)0);

				hils_cmd.motor_id = 3; hils_cmd.angle = -yaw_fin_deg;   xQueueSend(xAngleQueue, &hils_cmd, (TickType_t)0);



				// 모터 4-5 (동체)

				hils_cmd.motor_id = 4; hils_cmd.angle = cur_mis_pitch; xQueueSend(xAngleQueue, &hils_cmd, (TickType_t)0);

			    hils_cmd.motor_id = 5; hils_cmd.angle = cur_mis_yaw; xQueueSend(xAngleQueue, &hils_cmd, (TickType_t)0);
				//

				// 최종 날개 제어각을 큐에 넣음
				tx_to_queue.pitch_fin_deg = pitch_fin_deg;
				tx_to_queue.yaw_fin_deg = yaw_fin_deg;

	            // 큐로 제어 데이터 push
	            if (xQueueSend(xControlOutQueue, &tx_to_queue, 0) != pdPASS)
	            {
	                // 큐가 가득 찼다면 가장 오래된 게 의미 없을 가능성이 높으므로
	                // 오래된 걸 버리고 새 걸 넣고 싶다면 xQueueOverwrite (큐를 QueueSet 아닌 StreamBuffer로 만드는 방식도 가능)
					if (xSemaphoreTake(printMutex, portMAX_DELAY) == pdTRUE) {
						printf("control_task: queue full\r\n");
						xSemaphoreGive(printMutex);
					}
	            }
			}
	    }
}
