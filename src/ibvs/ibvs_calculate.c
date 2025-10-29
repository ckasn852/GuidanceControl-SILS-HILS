#include "ibvs_calculate.h"

#include <math.h>   // fabsf
#include <stdio.h>

#define CENTER_X 0.0f
#define CENTER_Y 0.0f

#define LAMBDA   3000.0f

#define LM_DAMP_INIT       1e-6f
#define LM_DAMP_MAX        1e-2f
#define DIST_MIN_CLAMP        1e-6f

float Jacobian[2][6];
float Jacobian_Transpose[6][2];
float Jacobian_Pseudo_Inverse[6][2];
float M[2][2];
float M_inv[2][2];
float Cur_Error[2] = { 0.0f, 0.0f };

float M_damped[2][2];
float mu = LM_DAMP_INIT;
int ok = -1;

static void create_jacobian(float x, float y, float Z, float Jacobian[2][6]) {

    Jacobian[0][0] = -1.0f / Z;
    Jacobian[0][1] = 0.0f;
    Jacobian[0][2] = x / Z;
    Jacobian[0][3] = x * y;
    Jacobian[0][4] = -(1.0f + x * x);
    Jacobian[0][5] = y;

    Jacobian[1][0] = 0.0f;
    Jacobian[1][1] = -1.0f / Z;
    Jacobian[1][2] = y / Z;
    Jacobian[1][3] = 1.0f + y * y;
    Jacobian[1][4] = -x * y;
    Jacobian[1][5] = -x;
}

static void transpose_2x6(const float Jacobian[2][6], float Jacobian_Transpose[6][2]) {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 6; ++j) {
            Jacobian_Transpose[j][i] = Jacobian[i][j];
        }
    }
}

static void multiply_2x6_by_6x2(const float Jacobian[2][6], const float Jacobian_Transpose[6][2], float M[2][2]) {
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            float acc = 0.0f;
            for (int k = 0; k < 6; ++k) {
                acc += Jacobian[i][k] * Jacobian_Transpose[k][j];
            }
            M[i][j] = acc;
        }
    }
}

static int invert_2x2(const float M[2][2], float M_inv[2][2]) {
    const float a = M[0][0], b = M[0][1];
    const float c = M[1][0], d = M[1][1];

    const float det = a * d - b * c;

    if (fabsf(det) < 1e-12f) return -1;

    const float inv_det = 1.0f / det;
    M_inv[0][0] = d * inv_det;
    M_inv[0][1] = -b * inv_det;
    M_inv[1][0] = -c * inv_det;
    M_inv[1][1] = a * inv_det;
    return 0;
}

// 6x2 = 6x2 * 2x2  (Jacobian_Pseudo_Inverse = Jacobian_Transpose * M_inv)
static void multiply_6x2_by_2x2(const float Jacobian_Transpose[6][2], const float M_inv[2][2], float Jacobian_Pseudo_Inverse[6][2]) {
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 2; ++j) {
            Jacobian_Pseudo_Inverse[i][j] = Jacobian_Transpose[i][0] * M_inv[0][j] + Jacobian_Transpose[i][1] * M_inv[1][j];
        }
    }
}

// 6x1 = 6x2 * 2x1  (Vc = Jacobian_Pseudo_Inverse * b)
static void multiply_6x2_by_2x1(const float Jacobian_Pseudo_Inverse[6][2], const float Cur_Error[2], float Vc[6]) {
    /*
    Vc[0] : X, Vc[1] : Y, Vc[2] : Z
    Vc[3] : Pitch, Vc[4] : Yaw, Vc[5] : Roll  */
    for (int i = 0; i < 6; ++i) {
        Vc[i] = Jacobian_Pseudo_Inverse[i][0] * Cur_Error[0] + Jacobian_Pseudo_Inverse[i][1] * Cur_Error[1];
            // printf("Vc[%d] : %.8f, Jacobian_Pseudo_Inverse[i][0] : %.8f, Jacobian_Pseudo_Inverse[i][1] %.8f, Cur_Error[0] : %.8f, Cur_Error[1] : %.8f \n",
            // i , Vc[i], Jacobian_Pseudo_Inverse[i][0], Jacobian_Pseudo_Inverse[i][1], Cur_Error[0], Cur_Error[1]);
    }
}

void ibvs_calculate(float img_x, float img_y, float dist, float* velocity_out) {
    const float error[2] = { img_x - CENTER_X, img_y - CENTER_Y };

    create_jacobian(error[0], error[1], dist, Jacobian);
    transpose_2x6(Jacobian, Jacobian_Transpose);
    multiply_2x6_by_6x2(Jacobian, Jacobian_Transpose, M);

    for (;;) {
        // M_damped = M + mu * I
        M_damped[0][0] = M[0][0] + mu;
        M_damped[0][1] = M[0][1];
        M_damped[1][0] = M[1][0];
        M_damped[1][1] = M[1][1] + mu;

        ok = invert_2x2(M_damped, M_inv);
        if (ok == 0) break;
        if (mu >= LM_DAMP_MAX) break;
        mu *= 10.0f;
    }

    if (ok != 0) {
        for (int i = 0; i < 6; ++i) velocity_out[i] = 0.0f;
        return;
    }

    multiply_6x2_by_2x2(Jacobian_Transpose, M_inv, Jacobian_Pseudo_Inverse);

    Cur_Error[0] = LAMBDA * error[0];
    Cur_Error[1] = LAMBDA * error[1];

//    printf("Error[0] : %.4f, Error[1] : %.4f\n", error[0], error[1]);
//    printf("Cur_Error_X : %.4f, Cur_Error_Y : %.4f\n", Cur_Error[0], Cur_Error[1]);

    multiply_6x2_by_2x1(Jacobian_Pseudo_Inverse, Cur_Error, velocity_out);
}
