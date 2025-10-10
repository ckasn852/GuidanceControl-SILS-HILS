#include "ibvs_calculate.h"

#define CENTER_X 0                          // �̹��� ���� X ��ǥ
#define CENTER_Y 0                           // �̹��� ���� Y ��ǥ
#define LAMBDA 1.0f							// ��� �̵�(�ʱⰪ �ϴ� 1)

static float cur_target_coords[2];			// ���� Ÿ���� [x,y] ��ǥ
static float cur_target_error[2];           // ���� [x,y] ��ǥ ����
static float jacobian_original[2*6];        // ���ں�� ��� ����
static float jacobian_inverse[6*6];         // ���ں�� ����� ����
static float jacobian_transpose[6*2];		// ���ں�� ��ġ��� ����
static float jacobian_mutil[6*6];			// ���� ����� ����� ���� ��
static float jacobian_pinverse[6*2];			// ���ں�� ���翪��� ����


void calculate_target_coords_error(float target_x, float target_y){
	// IBVS�� ���ŵ� Ÿ�� ��ǥ�� ����ü�� �Է�
	cur_target_coords[0] = target_x;
	cur_target_coords[1] = target_y;

	// ���� ���
	cur_target_error[0] = target_x - CENTER_X;
	cur_target_error[1] = target_y - CENTER_Y;
}

// IBVS ��� �Լ�
int IBVS_calculation(float* velocity_out, float x, float y, float Z ){
	creat_jacobian(jacobian_original, x, y, Z);										// ��� �����(2x6)
	matrix_transpose(jacobian_original,jacobian_transpose, 2, 6);					// ��ġ��� ���(6x2)
	matrix_mul(jacobian_transpose, jacobian_original, jacobian_mutil, 6, 2, 6);  	// A^T(��ġ���) * A = (6x6)
	int is_invertible = matrix_inverse(jacobian_mutil, jacobian_inverse, 6);		// ����� ���(6x6)

	if(is_invertible == -1 ) return -1;
	matrix_mul(jacobian_inverse, jacobian_transpose, jacobian_pinverse, 6, 6, 2); 	// ����� * A^T(��ġ���)(6x2)

	//���� IBVS ��� ���̳ʽ� ���� * ���ں�� ���翪��� * ����
	// ���ں�� ���翪���(6x2) * ����(2x1) = (6x1)
	matrix_mul(jacobian_pinverse, cur_target_error,velocity_out, 6, 2, 1);
	// ��� �̵�(����) ��
	matrix_scalar_mul(-LAMBDA, velocity_out, 6, 1);

	return 0;
}
