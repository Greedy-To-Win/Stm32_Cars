#ifndef __PID_H__
#define __PID_H__
#include "sys.h"
//位置式PID结构体
typedef struct 
{
	float Kp;
	float Ki;
	float Kd;
	float Out;
	float Now_Date;
	float Target_Date;
	float Error;
	float Last_Error;
	float Error_sum;
	float Kd_date;//角度环的微分为角速度，不需要Error-LastError
	uint8_t timer;
}Position_PID_t;
//增量式PID结构体
typedef struct 
{
	float Kp;
	float Ki;
	float Kd;
	float Out;
	float Out_Inc;
	float Now_Date;
	float Target_Date;
	float Error;
	float Last_Error;
	uint8_t timer;
}Inc_PID_t;



extern Position_PID_t Angle_loop;
extern Position_PID_t Angle_speed_loop;
extern Position_PID_t Turn_loop;
extern Inc_PID_t Motor_Velocity_loop;



void Inc_PID_Control(Inc_PID_t* Inc_PID,uint16_t Max_Output);
void Position_PID_Control(Position_PID_t* Position_PID,uint16_t Max_Output,uint16_t Max_i_date,uint8_t Using_kd_date);
void Turn_PID_Control(Position_PID_t* Turn_PID,uint16_t Max_Output,uint8_t Using_kd_date);

void Motor_Velocity_Set(int16_t speed);
#endif
