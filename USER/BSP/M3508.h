#ifndef __M3508_H
#define __M3508_H
#include "stdint.h"

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制
#define RPM_2_ANGLE_PER_SEC 6        // 1圈=6个编码器值
/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度
    float speed_aps;          // 角速度,单位为:度/秒
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度 Celsius

    float total_angle;   // 总角度,注意方向
    int32_t total_round; // 总圈数,注意方向
} DJI_Motor_Measure_s;
typedef struct
{
   	float kp, ki, kd; //三个系数
    float error, lastError; //误差、上次误差
    float integral, maxIntegral; //积分、积分限幅
    float output, maxOutput; //输出、输出限幅
}PID;
typedef struct
{
	PID inner;
	PID outer;
	float output;
}CascadePID;

void PID_Calc(PID *pid, float reference, float feedback);
void DecodeDJIMotor(uint8_t rxbuff[8],DJI_Motor_Measure_s* M3508);
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut);
void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb);
#endif