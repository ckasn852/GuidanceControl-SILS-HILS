#include "pid.h"
#include <stdio.h>
#define dt 0.03f
void pid_init(PID_t* pid, float Kp, float Ki, float Kd, float out_max, float out_min) {
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->integral = 0.0f;
	pid->pre_e = 0.0f;
	pid->out_max = out_max;
	pid->out_min = out_min;
}
float pid_calculation(PID_t* pid, float omega, float gyro) {
	// 오차(Error) 계산 및 스케일 보정 (IBVS Lambda 값과 싱크 맞춤)
	//e = (omega - gyro) * 0.001;
	float e = (omega - gyro) ;
	// P: 비례항 (Proportional)
	float proportional = pid->Kp * e;
	// D: 미분항 (Derivative)
	float derivative = pid->Kd * (e - pid->pre_e) / dt; // Kd 적용
	// I: 적분항 (Integral)
	float integral = pid->Ki * pid->integral;
	float after_pid = proportional + integral + derivative;
/*
//	// 디버깅용 출력
//	printf("PID total      : %.4f\n", after_pid);
// 	printf("PID proportional : %.4f\n", (pid->Kp * e));
//	printf("PID integral     : %.4f\n", (pid->Ki * pid->integral));
//	printf("PID derivative   : %.4f\n", (pid->Kd * derivative));
*/
	// PID 클램핑(제어 폭주 방지)
	if (after_pid > pid->out_max) {
		after_pid = pid->out_max;
	}
	else if (after_pid < pid->out_min) {
		after_pid = pid->out_min;
	}
	else {
		// 출력이 포화되지 않은 정상 범위에서만 적분을 수행
		pid->integral += e * dt;
	}
	// 현재 오차를 과거 오차로 저장
	pid->pre_e = e;
	return after_pid;
}
