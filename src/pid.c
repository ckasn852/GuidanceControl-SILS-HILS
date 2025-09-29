#include "pid.h"

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
	float e = omega - gyro;

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
