#include "pid.h"
#include <stdio.h>

float proportional;				// 비례항
float integral;					// 적분항
float derivative;				// 미분항
float e;						// 에러
float after_pid;

void pid_init(PID_t* pid, float Kp, float Ki, float Kd, float out_max, float out_min) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->integral = 0.0f;
	pid->pre_e = 0.0f;
	pid->out_max = out_max;
	pid->out_min = out_min;
}

float pid_calculation(PID_t* pid, float omega, float gyro, float dt) {

	// 오차(Error) 계산
	e = (omega - gyro);

	// PID (비례항, 적분항, 미분항) 계산
	proportional = pid->Kp * e;
	integral = pid->integral += e * dt;
	derivative = (e - pid->pre_e) / dt;

	// 최종 PID 출력 계산
	after_pid = proportional + pid->Ki * integral + pid->Kd * derivative;

//	// 디버깅용 출력
//	printf("PID total      : %.4f\n", after_pid);
// 	printf("PID proportional : %.4f\n", (pid->Kp * e));
//	printf("PID integral     : %.4f\n", (pid->Ki * pid->integral));
//	printf("PID derivative   : %.4f\n", (pid->Kd * derivative));

	// PID 클램핑(제어 폭주 방지)
	if (after_pid > pid->out_max) {
		after_pid = pid->out_max;
	}
	if (after_pid < pid->out_min) {
		after_pid = pid->out_min;
	}

	// 현재 오차를 과거 오차로 저장
	pid->pre_e = e;

	return after_pid;
}
