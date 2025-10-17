#include "ibvs_calculate.h"
#include "matrix_calculation.h"
#include "xil_printf.h"

#define CENTER_X 0                          // 이미지 중점 X 좌표
#define CENTER_Y 0                          // 이미지 중점 Y 좌표
#define LAMBDA 100000.0f							// 상수 이득(초기값 일단 1)

static float dot_error[2];			// 현재 타겟의 [x,y] 좌표
static float cur_target_error[2];           // 현재 [x,y] 좌표 오차
static float past_target_error[2];
static float cur_target_coords[2];

static float inverse_matrix[2*2];			// 새로운 역행렬


void calculate_target_coords_error(float target_x, float target_y){
	// IBVS로 수신된 타겟 좌표를 구조체에 입력
	cur_target_coords[0] = target_x;
	cur_target_coords[1] = target_y;

	// 편차 계산
	cur_target_error[0] = CENTER_X - cur_target_coords[0];
	cur_target_error[1] = CENTER_Y - cur_target_coords[1];

}

int IBVS_calculation(float* velocity_out, float x, float y){
	// 좌표 오차 계산
	calculate_target_coords_error(x, y);

	//2x2 matrix 역행렬 계산
	int inverse = inverse_2x2(inverse_matrix, x, y);

	if(inverse == -1){
		// 역행렬이 존재하지 않습니다!
		xil_printf(" det_a is ZERO. no inverse matrix!\r\n");
		 return -1;
	}

//	디버깅용 역행렬 확인
//	xil_printf("inverse_matrix\r\n");
//	print_matrix(inverse_matrix, 2, 2);
	// 각속도 계산(역행렬 * 오류)
	matrix_mul(inverse_matrix, cur_target_error, velocity_out, 2, 2, 1);
	matrix_scalar_mul(-LAMBDA, velocity_out, 2, 1);

	return 0;
}

