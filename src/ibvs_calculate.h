#ifndef __IBVS_H_
#define __IBVS_H_

#include "matrix_calculation.h"

void calculate_target_coords_error(float target_x, float target_y);
int IBVS_calculation(float* velocity_out, float x, float y, float Z );

#endif /*__IBVS_H_*/
