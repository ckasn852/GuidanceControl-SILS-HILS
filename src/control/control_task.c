
#define YAW_PITCH_TOLERANCE 0.3f // 오차 X, Y가 ±0.3 이내면 정지
#include "control_task.h"
#include "control_queue.h"
#include "../ibvs/ibvs_calculate.h"
#include "../pid/pid.h"
#include <math.h>

void control_task(){

	control_data rx_from_queue;
	control_output tx_to_queue;

	// PID 컨트롤러 인스턴스
	PID_t pid_yaw;
	PID_t pid_pitch;
	PID_Config_t pid_gain;

	float velocity_out[6]; 					// 목표 각속도 벡터
	float x, y, dist, cur_pitch, cur_yaw;	// 타겟 x,y 좌표(오차), 거리, 동체의 현재 Pitch, Yaw 각속도
	float target_pitch, target_yaw;			// Pitch, Yaw 방향 목표 각속도
	float pitch_fin_deg, yaw_fin_deg; 	// Pitch, Yaw 방향 목표 날개 제어량

	// PID Gain Pitch, Yaw 각각 적용
//	pid_init(&pid_pitch, 50.0f, 20.0f, 0.0001f, 200.0f, -200.0f);  // 50ms 최적 PID 설정
//  pid_init(&pid_yaw, 50.0f, 20.0f, 0.0001f, 200.0f, -200.0f);
	pid_config_init(&pid_gain, 25.0f, 10.0f, 0.0001f, 150.0f, -150.0f); // 30ms 최적 PID 설정
    pid_init(&pid_pitch, &pid_gain);
    pid_init(&pid_yaw, &pid_gain);

	while(1)
	    {
			// printf("control_task\n");
			// 데이터가 올 때까지 대기
			if (xQueueReceive(xControlQueue, &rx_from_queue, pdMS_TO_TICKS(100)) == pdTRUE)
			{
				// 수신 데이터를 지역 변수에 할당
				x = rx_from_queue.img_x,
				y = rx_from_queue.img_y;
				dist = rx_from_queue.distance;
				cur_yaw = rx_from_queue.yaw_rate;
				cur_pitch = rx_from_queue.pitch_rate;

				printf("control_task got data: x: %.4f y: %.4f dist: %.4f pitch: %.4f yaw: %.4f\r\n", x, y, dist, cur_pitch, cur_yaw);

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
				printf("Target pitch: %.4f, Target yaw: %.4f\r\n", velocity_out[3], velocity_out[4]);

				// PID 연산 후 Yaw, Pitch 날개 각도 제어 명령 출력
				if(target_pitch != 0){
					printf("[Pitch PID]\n");
					pitch_fin_deg = pid_calculation(&pid_pitch, target_pitch, cur_pitch);
				}
				else{
					pitch_fin_deg = 0;
				}
				if(target_yaw != 0){
					printf("[Yaw PID]\n");
					yaw_fin_deg = pid_calculation(&pid_yaw, target_yaw, cur_yaw);
				}
				else{
					yaw_fin_deg = 0;
				}

				// 최종 날개 제어각을 큐에 넣음
				tx_to_queue.pitch_fin_deg = pitch_fin_deg;
				tx_to_queue.yaw_fin_deg = yaw_fin_deg;

	            // 큐로 제어 데이터 push
	            if (xQueueSend(xControlOutQueue, &tx_to_queue, 0) != pdPASS)
	            {
	                // 큐가 가득 찼다면 가장 오래된 게 의미 없을 가능성이 높으므로
	                // 오래된 걸 버리고 새 걸 넣고 싶다면 xQueueOverwrite (큐를 QueueSet 아닌 StreamBuffer로 만드는 방식도 가능)
	                printf("control_task: queue full\r\n");
	            }
			}
	        vTaskDelay(pdMS_TO_TICKS(200)); // 예시
	    }
}
