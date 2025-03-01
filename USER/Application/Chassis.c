#include "BSP_CAN.h"
#include "M3508.h"
#include "Chassis.h"															
#include "stdint.h"
#include "PS2.h"

extern uint16_t XY[];
extern DJI_Motor_Measure_s M3508_lf,M3508_lb,M3508_rf,M3508_rb;
#define CHASSIS_WHEEL_OFFSET 0.7071f

int16_t Vx, Vy, Vw;
int16_t V_LF,V_LB,V_RF,V_RB;
//extern CascadePID mypid;

void GetMecanumWheelSpeed()
{
    Vw=XY[0]/11;
    Vy=XY[1]/11;                                          //左右偏移
    Vx=XY[2]/11;
    V_LF = (-Vx + Vy) * CHASSIS_WHEEL_OFFSET + Vw;
    V_RF = (Vx + Vy) * CHASSIS_WHEEL_OFFSET + Vw;
    V_RB = (-Vx - Vy) * CHASSIS_WHEEL_OFFSET + Vw;
    V_LB = (Vx - Vy) * CHASSIS_WHEEL_OFFSET + Vw;
}
//int16_t num_abs(int16_t x){
//    return x>0?x:-x;
//}
//void Scale_examine(int16_t *a,int16_t *b,int16_t *c,int16_t *d){
//    int maxval =0;
//    int values[4]={num_abs(*a),num_abs(*b),num_abs(*c),num_abs(*d)};
//    for(int i=0;i<4;i++){
//        if(values[i]>maxval){
//            maxval=values[i];
//        }
//    }
//    if(maxval>8191){
//        *a=*a/(maxval/8191);
//        *b=*b/(maxval/8191);
//        *c=*c/(maxval/8191);
//        *d=*d/(maxval/8191);
//    }
//}
void Chassis_Control(PID* lf,PID* lb,PID* rf,PID* rb){
	PID_Calc( lf, V_LF,0);
	PID_Calc( lb, V_LB,0);
  PID_Calc( rf, V_RF,0);
  PID_Calc( rb, V_RB,0);
	CAN_Transmit_lower(lf->output,rf->output,rb->output,lb->output);
}

void Dipan_Control(PID* pid_lf,PID* pid_lb,PID* pid_rf,PID* pid_rb){
    GetMecanumWheelSpeed();
//    Scale_examine(&V_LF,&V_LB,&V_RF,&V_RB);
    Chassis_Control(pid_lf,pid_lb,pid_rf,pid_rb);
}