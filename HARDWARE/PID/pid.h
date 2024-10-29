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
	float		 Error_Max;//偏差最大值
	float		 Error_temp;//偏差绝对值中介，//用于判断是否积分
	float		 Error_Min;//偏差最小值
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
	float		 Error_Max;//偏差最大值
	float		 Error_temp;//偏差绝对值中介，//用于判断是否积分
	float		 Error_Min;//偏差最小值
	uint8_t timer;
}Inc_PID_t;

typedef struct
{
	
	volatile float      Error1; // Error[n-1]
	volatile float      Error2; // Error[n-2]
	volatile float		 Error_sum;//积分值
	volatile float		 iIncpid;//PID输出值
	volatile float    inc_iError;//Error[n]增量
	volatile float    inc_Error1;//Error[n-1]增量
	volatile float		 Error_abs_Max;//偏差绝对值最大值
	volatile float		 Error_abs_Mid;//偏差绝对值最中值
	volatile float		 Error_abs_Min;//偏差绝对值最小值
	float control_parameter;//输出控制参数
} Exper_Inc_Pid_TCB_t;


extern Exper_Inc_Pid_TCB_t	 Exper_Motor_Velocity_TCB;//增量式专家控制块
extern Position_PID_t 	 Angle_loop;
extern Position_PID_t 	 Angle_speed_loop;
extern Position_PID_t 	 Turn_loop;
extern Inc_PID_t 		 Motor_Velocity_loop;



void Inc_PID_Control(Inc_PID_t* Inc_PID,uint16_t Max_Output);
void Position_PID_Control(Position_PID_t* Position_PID,uint16_t Max_Output,uint16_t Max_i_date,uint8_t Using_kd_date);
void Turn_PID_Control(Position_PID_t* Turn_PID,uint16_t Max_Output,uint8_t Using_kd_date);

//联合专家控制块的增量式专家PID控制器
void Inc_PID_Control_Expert(Inc_PID_t* Inc_PID,Exper_Inc_Pid_TCB_t* Exper_Motor_Velocity_TCB);

void Motor_Velocity_Set(int16_t speed);
#endif
