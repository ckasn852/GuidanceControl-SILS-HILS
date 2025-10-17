#ifndef __PID_H_
#define __PID_H_

typedef struct {
	float Kp, Ki, Kd;
	float pre_e;
	float integral;
	float out_max, out_min;
}PID_t;

typedef struct {
	float Kp, Ki, Kd;
	float out_max, out_min;
}PID_Config_t;

void pid_config_init(PID_Config_t* pid_set ,float Kp, float Ki, float Kd, float out_max, float out_min);
void pid_init(PID_t* pid, PID_Config_t* pid_set);
float pid_calculation(PID_t* pid, float omega, float gyro);


#endif /*__PID_H_ */
