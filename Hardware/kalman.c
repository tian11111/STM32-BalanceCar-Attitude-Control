#include "kalman.h"

void Kalman_Init(Kalman_t *k)
{
    if (k == 0) return;

    k->angle = 0.0f;
    k->bias  = 0.0f;
    k->rate  = 0.0f;

    k->P[0][0] = 1.0f;  k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;  k->P[1][1] = 1.0f;

    k->Q_angle   = 0.001f;
    k->Q_bias    = 0.003f;
    k->R_measure = 0.03f;
}

void Kalman_SetParams(Kalman_t *k, float Q_angle, float Q_bias, float R_measure)
{
    if (k == 0) return;
    k->Q_angle   = Q_angle;
    k->Q_bias    = Q_bias;
    k->R_measure = R_measure;
}

void Kalman_SetAngle(Kalman_t *k, float angle_deg)
{
    if (k == 0) return;
    k->angle = angle_deg;
}

float Kalman_Update(Kalman_t *k, float accAngle_deg, float gyroRate_dps, float dt_s)
{
    if (k == 0) return accAngle_deg;
    if (dt_s <= 0.0f) return k->angle;

    // 1) 预测（Predict）
    k->rate = gyroRate_dps - k->bias;
    k->angle += dt_s * k->rate;

    // 协方差矩阵预测
    k->P[0][0] += dt_s * (dt_s * k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt_s * k->P[1][1];
    k->P[1][0] -= dt_s * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt_s;

    // 2) 更新（Update）
    // 残差
    float y = accAngle_deg - k->angle;

    // 残差协方差
    float S = k->P[0][0] + k->R_measure;

    // 卡尔曼增益
    float K0 = k->P[0][0] / S;
    float K1 = k->P[1][0] / S;

    // 更新角度与零偏
    k->angle += K0 * y;
    k->bias  += K1 * y;

    // 更新协方差矩阵
    float P00 = k->P[0][0];
    float P01 = k->P[0][1];

    k->P[0][0] -= K0 * P00;
    k->P[0][1] -= K0 * P01;
    k->P[1][0] -= K1 * P00;
    k->P[1][1] -= K1 * P01;

    return k->angle;
}
