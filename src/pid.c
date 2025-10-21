#include <stdio.h>
#include <stdint.h>
#include "pid.h"

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

//PID 제어 (pid이름, 목표 각속도, 현재 각속도)
float pid_calculation(PID_t* pid, float target, float current) {
	float e = target - current;

	pid->integral += e * 0.01;
	float derivative = (e - pid->pre_e) / 0.01;

	float gyro_pid = 0;
	gyro_pid = pid->Kp * e
				+ pid->Ki * pid->integral
				+ pid->Kd * derivative;
	if (gyro_pid > pid->out_max) gyro_pid = pid->out_max;
	if (gyro_pid < pid->out_min) gyro_pid = pid->out_min;

	pid->pre_e = e;

	return gyro_pid;
}
