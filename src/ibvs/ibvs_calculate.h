#ifndef IBVS_CALCULATE_H_
#define IBVS_CALCULATE_H_

/**
 * @param img_x         [입력] 타겟의 이미지 x 좌표
 * @param img_y         [입력] 타겟의 이미지 y 좌표
 * @param dist          [입력] 타겟까지의 거리 (Z)
 * @param velocity_out  [출력] 계산된 6차원 속도(vx, vy, vz, wx, wy, wz)를 저장할 배열
 */
void ibvs_calculate(float img_x, float img_y, float dist, float* velocity_out);

#endif /* IBVS_CALCULATE_H_ */
