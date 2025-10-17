#define _CRT_SECURE_NO_WARNINGS

#include <math.h>
#include <stdio.h>
#include "matrix_calculation.h"
#include "xil_printf.h"

#define MAX_SIZE 6
static float augmented_matrix[MAX_SIZE][2*MAX_SIZE];


//디버깅용 행렬 출력함수
void print_matrix(float* matrix, int row, int col){
	int i, j;
	char str_num[10];
	for(i =0; i<row; i++){
		for(j = 0; j<col; j++){
			sprintf(str_num, "%.2f", matrix[i*col +j]);
			xil_printf("%s\t", str_num);
		}
		xil_printf("\r\n");
	}
}

//2x2 역행렬 계산 함수
int inverse_2x2(float* inverse, float x, float y){
//	char str_num[10];
	float AD = -x*x*y*y;
	float BC = -(1.0f+x*x)*(1.0f+y*y);
	float det_A = AD - BC;
//	sprintf(str_num, "%.2f", det_A);
//	xil_printf("%s\r\n", str_num);
	if(det_A == 0) return -1;


	inverse[0] = -x*y;
	inverse[1] = (1.0f+x*x);
	inverse[2] = -(1.0f+y*y);
	inverse[3] = x*y;



	matrix_scalar_mul(1/det_A, inverse, 2, 2);

	return 0;
}

//행렬 곱 함수 matrix1(raw1*col1)*matrix2(col1*col2)
void matrix_mul(float* matrix1, float* matrix2, float* mul_matrix, int row1, int col1, int col2){
	int i, j, k;
	for(i=0; i<row1;i++){
		for(j = 0; j <col2; j++){
			float sum =0.0f;
			for(k = 0; k < col1; k++){
				sum += matrix1[col1*i + k]*matrix2[col2*k + j];
			}
			mul_matrix[col2*i + j] = sum;
		}
	}
//	xil_printf("Multiply matrix\r\n");
//	print_matrix(mul_matrix, row1,col2);

}
// 스칼라곱 함수 (상수, 행렬, 행, 렬)
void matrix_scalar_mul(float a, float* matrix, int row, int col){
	int i, j;
	for(i = 0; i <row; i++){
		for(j =0; j < col; j++){
			matrix[i*col +j] *= a;
		}
	}
//	xil_printf("Scalar Multiply matrix\r\n");
//	print_matrix(matrix, row,col);
}
