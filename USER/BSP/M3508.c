#include "M3508.h"
#include <stdint.h>
#include "BSP_CAN.h"

/**
 * @todo  是否可以简化多圈角度的计算？
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
   DJI_Motor_Measure_s M3508,M3508_lf,M3508_lb,M3508_rf,M3508_rb;
   CascadePID mypid = {0}; //创建串级PID结构体变量
   PID pid_lf={0},pid_lb={0},pid_rf={0},pid_rb={0};

 /**
 * 解码DJIMotor数据
 * 该函数解析来自DJIMotor的8字节接收缓冲区数据，并更新电机测量值
 * 主要包括电角度、速度和电流的计算与平滑处理
 * 
 * @param rxbuff 接收缓冲区，包含8字节的电机反馈数据
 */
void DecodeDJIMotor(uint8_t rxbuff[8],DJI_Motor_Measure_s* M3508)
{
    // 指向M3508电机测量结构体
    DJI_Motor_Measure_s *measure = M3508;
    
    // 解析数据并对电流和速度进行滤波
    // 电机的反馈报文具体格式见电机说明手册
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    
    // 速度平滑处理
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    
    // 电流平滑处理
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    
    // 温度直接读取
    measure->temperature = rxbuff[6];

    // 多圈角度计算
    // 前提是假设两次采样间电机转过的角度小于180°,自己画个图就清楚计算过程了
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

/**
 * DJI电机丢失回调函数
 * 当失去与DJIMotor的连接时调用该函数，用于记录日志信息
 * 
 * @param motor_ptr 电机实例的指针，用于识别哪个电机丢失
 */
//首先定义PID结构体用于存放一个PID的数据
 
//用于初始化pid参数的函数
void PID_Init(PID *pid, float p, float i, float d, float maxI, float maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
}
 
//进行一次pid计算
//参数为(pid结构体,目标值,反馈值)，计算结果放在pid结构体的output成员中
void PID_Calc(PID *pid, float reference, float feedback)
{
 	//更新数据
    pid->lastError = pid->error; //将旧error存起来
    pid->error = reference - feedback; //计算新error
    //计算微分
    float dout = (pid->error - pid->lastError) * pid->kd;
    //计算比例
    float pout = pid->error * pid->kp;
    //计算积分
    pid->integral += pid->error * pid->ki;
    //积分限幅
    if(pid->integral > pid->maxIntegral) pid->integral = pid->maxIntegral;
    else if(pid->integral < -pid->maxIntegral) pid->integral = -pid->maxIntegral;
    //计算输出
    pid->output = pout+dout + pid->integral;
    //输出限幅
    if(pid->output > pid->maxOutput) pid->output =   pid->maxOutput;
    else if(pid->output < -pid->maxOutput) pid->output = -pid->maxOutput;
}
    void PID_CascadeCalc(CascadePID *pid, float outerRef, float outerFdb, float innerFdb)
{
	PID_Calc(&pid->outer,outerRef,outerFdb);
	PID_Calc(&pid->inner,pid->outer.output,innerFdb);
	pid->output = pid->inner.output;
}                  