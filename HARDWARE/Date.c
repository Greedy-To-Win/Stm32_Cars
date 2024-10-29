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
	Angle_speed_loop.Error_Max=3999;
	Angle_speed_loop.Error_Min=-3999;
	Angle_speed_loop.Error_temp=2999;//用于判断是否积分
	/*角度PID环控制参数初始化*/
	Angle_loop.Kp=-200;
	Angle_loop.Kd=-0.35;
	Angle_loop.Target_Date=0;
	Angle_loop.Now_Date=0;
	Angle_loop.Error_Max=209;
	Angle_loop.Error_Min=-209;
	Angle_loop.Error_temp=0;//用于判断是否积分，此处不需要
	/*转向PID环控制参数初始化*/
	Turn_loop.Kp=20;
	Turn_loop.Target_Date=0;
//	Turn_loop.Error_Max=3999;
//	Turn_loop.Error_Min=-3999;
//	Turn_loop.Error_temp=2999;
/**************增量式**************/
	/*速度PID环控制参数初始化*/				
	Motor_Velocity_loop.Kp=-0.35;
	Motor_Velocity_loop.Ki=-152;
	Motor_Velocity_loop.Target_Date=0;
	Motor_Velocity_loop.Now_Date=0;
	Motor_Velocity_loop.Error_Max=70;//起步时最大单电机35，误差0-35*2=-70
	Motor_Velocity_loop.Error_Min=-70;
	Motor_Velocity_loop.Error_temp=0;//用于判断是否积分，此处不需要
	
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
