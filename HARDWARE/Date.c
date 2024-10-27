#include "Date.h"

//变速PID数据
PID_Date_t Dynamic[30];
/**************************************************************************
函数功能：PID数据初始化
入口参数：无
返回  值：无
**************************************************************************/

void PID_Date_Init()
{
/**************位置式**************/
	/*角速度PID环控制参数初始化*/
	Angle_speed_loop.Kp=1.92;
	Angle_speed_loop.Ki=0.6;
	Angle_speed_loop.Target_Date=0;
	Angle_speed_loop.Now_Date=0;
	
	/*角度PID环控制参数初始化*/
	Angle_loop.Kp=-200;
	Angle_loop.Kd=-0.35;
	Angle_loop.Target_Date=0;
	Angle_loop.Now_Date=0;
	
	/*角度PID环控制参数初始化*/
	Turn_loop.Kp=20;
	Turn_loop.Target_Date=0;
	
/**************增量式**************/
	/*速度PID环控制参数初始化*/				
	Motor_Velocity_loop.Kp=-0.35;
	Motor_Velocity_loop.Ki=-152;
	Motor_Velocity_loop.Target_Date=0;
	Motor_Velocity_loop.Now_Date=0;
	
}
void Motor_Date_Init()
{
	Blance_motor.Motor_out_max=5000;
	Blance_motor.Motor_state=Motor_Normal;

}
void Dynamic_PID_Control(uint16_t speed)
{
	if(0==speed)
	{
		
	}
	else
	{
		
		//PID参数调整
	}
}
