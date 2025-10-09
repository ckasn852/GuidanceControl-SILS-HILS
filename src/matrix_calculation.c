#include "matrix_calculation.h"

#define MAX_SIZE 6
static float augmented_matrix[MAX_SIZE][2*MAX_SIZE];

// ��� ���Ǹ� ���� 1���� ��� ���
void creat_jacobian(float* jacob, float x, float y, float Z){
	 // 1�� ���� ä���
	jacob[0] = -1.0f / Z;
	jacob[1] = 0.0f;
	jacob[2] = x / Z;
	jacob[3] = x * y;
	jacob[4] = -(1.0f + x * x);
	jacob[5] = y;

	// 2�� ���� ä���
	jacob[6] = 0.0f;
	jacob[7] = -1.0f / Z;
	jacob[8] = y / Z;
	jacob[9] = 1.0f + y * y;
	jacob[10] = -x * y;
	jacob[11] = -x;
}

void matrix_transpose(float* jacob, float* jacob_transpose, int row, int col){
	int i, j;
	for(i = 0; i <row; i++){
		for(j = 0; j <col; j++){
			jacob_transpose[j*row + i] = jacob[i*col + j];
		}
	}
}

//��� �� �Լ� matrix1(raw1*col1)*matrix2(col1*col2)
void matrix_mul(float* matrix1, float* matrix2, float* mul_matrix, int row1, int col1, int col2){
	int i, j, k;
	for(i=0; i<row1;i++){
		for(j = 0; j <col2; j++){
			float sum;
			for(k = 0; k < col1; k++){
				sum += matrix1[col1*i + k]*matrix2[col2*k + j];
			}
			mul_matrix[col2*i + j] = sum;
		}
	}

}
// ��Į��� �Լ� (���, ���, ��, ��)
void matrix_scalar_mul(float a, float* matrix, int row, int col){
	int i, j;
	for(i = 0; i <row; i++){
		for(j =0; j < col; j++){
			matrix[i*col +j] *= a;
		}
	}
}

// ����� �Լ�
int matrix_inverse(float* matrix, float* inverse_matrix, int size){
	// Ȯ�� ��� [A|I]
	int i,j,k;
	for(i = 0; i <size; i++){
		for(j = 0; j <size; j++){
			augmented_matrix[i][j] = matrix[i*size +j];
		}
		//���� ��� ���̱�
		for(j = size; j <2*size; j++){
			augmented_matrix[i][j] = ((j - size) == i)? 1.0f : 0.0f;
		}
	}

	// ���콺 - ���� �ҰŹ�
	for(i = 0; i< size; i++){
		//������� ���� �� �� ���� ���
		if(fabs(augmented_matrix[i][i]) < 1e-6f ) return -1;

		float pivot = augmented_matrix[i][i];
		//pivot 1�� �����
		for(j = 0; j <size*2 ; j++){
			augmented_matrix[i][j] /= pivot;
		}

		//���� ����� ���� ��ķ� ����
		for(k =0; k < size ; k ++){
			if(k == i) continue;

			float ratio = augmented_matrix[k][i];
			for(j = 0; j <2*size; j++){
				augmented_matrix[k][j] -= ratio * augmented_matrix[i][j];
			}
		}
	}
	// Ȯ��� ��� ���� ��
	for(i = 0; i<size; i++){
		for(j = 0; j <size; j++){
			inverse_matrix[i*size + j] = augmented_matrix[i][j+size];
		}
	}
	return 0;
}

