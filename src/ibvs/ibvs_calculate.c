// ibvs_calculate.c
// 출력: velocity_out[3]:pitch, velocity_out[4]:yaw만 사용. 나머지 0.
// LAMBDA 조정 필요(제어량 조절 시)

#include "ibvs_calculate.h"
#include <stdbool.h>
#include <stdio.h>
#include <float.h>

#ifndef CENTER_X
#define CENTER_X 0.0f
#endif
#ifndef CENTER_Y
#define CENTER_Y 0.0f
#endif
#ifndef LAMBDA
#define LAMBDA 100.0f
#endif

// NaN/Inf 체크
static inline bool is_finitef(float x) {
    if (!(x == x)) return false;       // NaN
    float ax = (x >= 0.0f) ? x : -x;   // |x|
    return (ax <= FLT_MAX);            // Inf 방지
}

/*
 * 메인: 회전 2x2만 사용
 * velocity_out[0]:x
 * velocity_out[1]:y
 * velocity_out[2]:z
 * velocity_out[3]:pitch
 * velocity_out[4]:yaw
 * velocity_out[5]:Roll
*/
void ibvs_calculate(float img_x, float img_y, float /*dist_unused*/, float* velocity_out)
{
    if (!velocity_out) return;

    // 출력 초기화
    for (int i = 0; i < 6; ++i) velocity_out[i] = 0.0f;

    // 에러 벡터 e = [x; y]
    float x = img_x - CENTER_X;
    float y = img_y - CENTER_Y;

    // 유효성 가드 (문제 있는 입력이면 0 반환)
    if (!is_finitef(x) || !is_finitef(y)) {
        return;
    }

    // 회전 성분 자코비안 Lw (2x2), 포인트 특징(정규화 좌표) 기준
    // Lw = [ x*y,       -(1+x^2)
    //        1+y^2,     -x*y     ]
    float a = x * y;
    float b = -(1.0f + x * x);
    float c = 1.0f + y * y;
    float d = -x * y;

    // det(Lw) = 1 + x^2 + y^2  (항상 > 0)
    float det = 1.0f + x * x + y * y;

    // 수치 가드 (이론상 >0 이지만 극단치 방지)
    if (!(det > 0.0f)) {
        return;
    }

    float inv_det = 1.0f / det;

    // b = LAMBDA * e
    float bx = LAMBDA * x;
    float by = LAMBDA * y;

    // [omegax; omegay] = Lw^{-1} b = (1/det) * [ d  -b; -c  a ] * [bx; by]
    float omegax = ( d * bx - b * by) * inv_det;  // Pitch
    float omegay = (-c * bx + a * by) * inv_det;  // Yaw

    // 출력 반영 (나머지는 0 유지)
    velocity_out[3] = omegax;   // Pitch
    velocity_out[4] = omegay;   // Yaw
}
