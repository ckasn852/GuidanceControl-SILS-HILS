#include "ibvs_calculate.h"
#include "matrix_calculation.h"
#include "xil_printf.h"

#define CENTER_X 0                          // �̹��� ���� X ��ǥ
#define CENTER_Y 0                          // �̹��� ���� Y ��ǥ
#define LAMBDA 100000.0f							// ��� �̵�(�ʱⰪ �ϴ� 1)

static float dot_error[2];			// ���� Ÿ���� [x,y] ��ǥ
static float cur_target_error[2];           // ���� [x,y] ��ǥ ����
static float past_target_error[2];
static float cur_target_coords[2];

static float inverse_matrix[2*2];			// ���ο� �����


void calculate_target_coords_error(float target_x, float target_y){
	// IBVS�� ���ŵ� Ÿ�� ��ǥ�� ����ü�� �Է�
	cur_target_coords[0] = target_x;
	cur_target_coords[1] = target_y;

	// ���� ���
	cur_target_error[0] = CENTER_X - cur_target_coords[0];
	cur_target_error[1] = CENTER_Y - cur_target_coords[1];

}

int IBVS_calculation(float* velocity_out, float x, float y){
	// ��ǥ ���� ���
	calculate_target_coords_error(x, y);

	//2x2 matrix ����� ���
	int inverse = inverse_2x2(inverse_matrix, x, y);

	if(inverse == -1){
		// ������� �������� �ʽ��ϴ�!
		xil_printf(" det_a is ZERO. no inverse matrix!\r\n");
		 return -1;
	}

//	������ ����� Ȯ��
//	xil_printf("inverse_matrix\r\n");
//	print_matrix(inverse_matrix, 2, 2);
	// ���ӵ� ���(����� * ����)
	matrix_mul(inverse_matrix, cur_target_error, velocity_out, 2, 2, 1);
	matrix_scalar_mul(-LAMBDA, velocity_out, 2, 1);

	return 0;
}

