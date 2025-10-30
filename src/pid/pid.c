#include "pid.h"
#include <stdio.h>
#define dt 0.01f

float proportional; // 비례항
float integral; // 적분항
float derivative; // 미분항
float e; // 에러
float after_pid;

void pid_config_init(PID_Config_t* pid_set ,float Kp, float Ki, float Kd, float out_max, float out_min){
	pid_set->Kp = Kp;
	pid_set->Ki = Ki;
	pid_set->Kd = Kd;
	pid_set->out_max = out_max;
	pid_set->out_min = out_min;
}

void pid_init(PID_t* pid, PID_Config_t* pid_set) {
	pid->Kp = pid_set->Kp;
	pid->Ki = pid_set->Ki;
	pid->Kd = pid_set->Kd;
	pid->integral = 0.0f;
	pid->pre_e = 0.0f;
	pid->out_max = pid_set->out_max;
	pid->out_min = pid_set->out_min;
}

float pid_calculation(PID_t* pid, float omega, float gyro) {

	// 오차(Error) 계산 및 스케일 보정 (IBVS Lambda 값과 싱크 맞춤)
	e = (omega - gyro) * 0.002;

	// PID (비례항, 적분항, 미분항) 계산
	proportional = pid->Kp * e;
	integral = pid->integral += e * dt;
	derivative = (e - pid->pre_e) / dt;

	// 최종 PID 출력 계산
	after_pid = proportional + integral + derivative;

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
