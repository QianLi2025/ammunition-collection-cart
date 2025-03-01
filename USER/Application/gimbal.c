//#include "gimbal.h"
//#include "BSP_CAN.h"
//#include "M3508.h"
//#include "Remote.h"

//extern CascadePID mypid;
//extern DJI_Motor_Measure_s M3508;
//extern RC_ctrl_t rc_ctrl[0];
//float  target_aps=0;

//void Gimbal_Control(){
//    target_aps += (rc_ctrl[0].rc.rocker_l_)/330;
//	PID_CascadeCalc( &mypid, target_aps, M3508.total_angle, M3508.speed_aps);
//	CAN_Transmit(&hcan1,0x1FF,0,0,0,mypid.output); 
//	CAN_Transmit(&hcan2,0x1FF,0,0,0,0);
//}