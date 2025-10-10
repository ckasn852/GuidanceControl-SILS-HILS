#include "ibvs_calculate.h"

#define CENTER_X 0                          // 이미지 중점 X 좌표
#define CENTER_Y 0                           // 이미지 중점 Y 좌표
#define LAMBDA 1.0f							// 상수 이득(초기값 일단 1)

static float cur_target_coords[2];			// 현재 타겟의 [x,y] 좌표
static float cur_target_error[2];           // 현재 [x,y] 좌표 오차
static float jacobian_original[2*6];        // 자코비안 행렬 선언
static float jacobian_inverse[6*6];         // 자코비안 역행렬 선언
static float jacobian_transpose[6*2];		// 자코비안 전치행렬 선언
static float jacobian_mutil[6*6];			// 유사 역행렬 계산을 위한 곱
static float jacobian_pinverse[6*2];			// 자코비안 유사역행렬 선언


void calculate_target_coords_error(float target_x, float target_y){
	// IBVS로 수신된 타겟 좌표를 구조체에 입력
	cur_target_coords[0] = target_x;
	cur_target_coords[1] = target_y;

	// 편차 계산
	cur_target_error[0] = target_x - CENTER_X;
	cur_target_error[1] = target_y - CENTER_Y;
}

// IBVS 계산 함수
int IBVS_calculation(float* velocity_out, float x, float y, float Z ){
	creat_jacobian(jacobian_original, x, y, Z);										// 행렬 만들기(2x6)
	matrix_transpose(jacobian_original,jacobian_transpose, 2, 6);					// 전치행렬 계산(6x2)
	matrix_mul(jacobian_transpose, jacobian_original, jacobian_mutil, 6, 2, 6);  	// A^T(전치행렬) * A = (6x6)
	int is_invertible = matrix_inverse(jacobian_mutil, jacobian_inverse, 6);		// 역행렬 계산(6x6)

	if(is_invertible == -1 ) return -1;
	matrix_mul(jacobian_inverse, jacobian_transpose, jacobian_pinverse, 6, 6, 2); 	// 역행렬 * A^T(전치행렬)(6x2)

	//최종 IBVS 계산 마이너스 람다 * 자코비안 유사역행렬 * 에러
	// 자코비안 유사역행렬(6x2) * 에러(2x1) = (6x1)
	matrix_mul(jacobian_pinverse, cur_target_error,velocity_out, 6, 2, 1);
	// 상수 이득(람다) 곱
	matrix_scalar_mul(-LAMBDA, velocity_out, 6, 1);

	return 0;
}
