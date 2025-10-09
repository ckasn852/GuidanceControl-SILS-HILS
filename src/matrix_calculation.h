#ifndef __MATRIX_H_
#define __MATRIX_H_

#include <math.h>

void creat_jacobian(float* jacob, float x, float y, float Z);
void matrix_transpose(float* jacob, float* jacob_transpose, int raw, int col);
void matrix_mul(float* matrix1, float* matrix2, float* mul_matrix, int raw1, int col1, int col2);
void matrix_scalar_mul(float a, float* matrix, int row, int col);
int matrix_inverse(float* matrix, float* inverse_matrix, int size);

#endif /*__MATRIX_H_*/
