#ifndef __KALMAN_H
#define __KALMAN_H

#include "stm32f10x.h"

#endif

typedef struct
{
    float angle;        // 当前估计角度 (deg)
    float bias;         // 估计陀螺零偏 (deg/s)
    float rate;         // 去零偏后的角速度 (deg/s)

    float P[2][2];      // 误差协方差矩阵

    // 噪声参数（可调）
    float Q_angle;      // 角度过程噪声
    float Q_bias;       // 零偏过程噪声
    float R_measure;    // 测量噪声（加速度角）
} Kalman_t;

/**
 * @brief  初始化卡尔曼滤波器（带默认参数）
 */
void Kalman_Init(Kalman_t *k);

/**
 * @brief  设置卡尔曼滤波参数
 */
void Kalman_SetParams(Kalman_t *k, float Q_angle, float Q_bias, float R_measure);

/**
 * @brief  强制设置滤波器角度（处理角度跨越/初始化用）
 */
void Kalman_SetAngle(Kalman_t *k, float angle_deg);

/**
 * @brief  运行一次更新
 * @param  accAngle_deg  加速度算出的角度（deg）
 * @param  gyroRate_dps  陀螺角速度（deg/s）
 * @param  dt_s          采样周期（秒）
 * @return 滤波后的角度（deg）
 */
float Kalman_Update(Kalman_t *k, float accAngle_deg, float gyroRate_dps, float dt_s);

