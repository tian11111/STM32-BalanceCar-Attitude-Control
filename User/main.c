#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "PWM.h"
#include "Motor.h"
#include "Encoder.h"
#include "kalman.h"
#include <math.h>

#define RAD2DEG 57.2957795f
#define GYRO_SENS_2000 16.4f // LSB/(deg/s) 角速度(deg/s) = 原始值 / 16.4
#define DT_MS 10

uint8_t ID;								//定义用于存放ID号的变量
int16_t AX, AY, AZ, GX, GY, GZ;			//定义用于存放各个数据的变量
Kalman_t kRoll;
Kalman_t kPitch;
float yaw_deg = 0.0f;
int enc_offset = 0;

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	MPU6050_Init();		//MPU6050初始化
	PWM_Init();
	Motor_Init();
	Encoder_Init();
  Kalman_Init(&kRoll);
  Kalman_Init(&kPitch);
	
	while (1)
	{
		MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);		
    float dt = DT_MS / 1000.0f;
    // ① 陀螺：原始 -> deg/s
    float gx = (float)GX / GYRO_SENS_2000;
    float gy = (float)GY / GYRO_SENS_2000;
		float gz = (float)GZ / GYRO_SENS_2000;
		
		float ax = (float)AX;
		float ay = (float)AY;
		float az = (float)AZ;
		if (gz > -2.0f && gz < 2.0f) gz = 0.0f;

    // roll/pitch 的加速度角（deg）
    float roll_acc  = atan2f(AY, AZ) * RAD2DEG;
    float pitch_acc = atan2f(-AX, sqrtf(AY*AY + AZ*AZ)) * RAD2DEG;

    // ③ 可选：处理 pitch 接近 ±90° 时的跳变
    if (fabsf(pitch_acc - kPitch.angle) > 90.0f)
    {
        Kalman_SetAngle(&kPitch, pitch_acc);
    }

    // ④ 卡尔曼融合
    float roll  = Kalman_Update(&kRoll,  roll_acc,  gx, dt);
    float pitch = Kalman_Update(&kPitch, pitch_acc, gy, dt);
		yaw_deg += gz * dt;

    // ⑤ OLED 显示
	  OLED_ShowString(1, 1, "x:");
    OLED_ShowSignedNum(1, 3, (int16_t)roll,  5);
		OLED_ShowString(2, 1, "y:");
    OLED_ShowSignedNum(2, 3, (int16_t)pitch, 5);
		OLED_ShowString(3, 1, "Z:");
		OLED_ShowSignedNum(3, 4, GZ, 6);
		OLED_ShowString(4, 1, "Yaw:");
		OLED_ShowNum(4, 5, (int16_t)(yaw_deg*10), 3);

		float yaw_180 = yaw_deg;
		if (yaw_180 > 180.0f) yaw_180 = 360.0f - yaw_180; 
		int yaw_step = (int)yaw_180 ;   // 0..18
		if (yaw_step < 0) yaw_step = 0;
		if (yaw_step > 18) yaw_step = 18;
		
		enc_offset += Encoder_Get();            
		if (enc_offset < -18) enc_offset = -18;
		if (enc_offset >  18) enc_offset =  18;
		
		int step = yaw_step + enc_offset;
		if (step < 0) step = 0;
		if (step > 18) step = 18;
		
		OLED_ShowNum(4, 10, step, 4);
		Motor_SetSpeed(step*5);
    Delay_ms(DT_MS);
	}
}
