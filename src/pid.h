#ifndef __PID_H_
#define __PID_H_

#include <stdio.h>
#include <stdint.h>

typedef struct {
	float Kp, Ki, Kd;
	float pre_e;
	float integral;
	float out_max, out_min;
}PID_t;

void pid_init(PID_t* pid, float Kp, float Ki, float Kd, float out_max, float out_min);
float pid_calculation(PID_t* pid, float omega, float gyro);


#endif /*__PID_H_ */
