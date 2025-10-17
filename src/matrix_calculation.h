#ifndef __MATRIX_H_
#define __MATRIX_H_

void print_matrix(float* matrix, int row, int col);
int inverse_2x2(float* inverse, float x, float y);
void matrix_mul(float* matrix1, float* matrix2, float* mul_matrix, int raw1, int col1, int col2);
void matrix_scalar_mul(float a, float* matrix, int row, int col);


#endif /*__MATRIX_H_*/
