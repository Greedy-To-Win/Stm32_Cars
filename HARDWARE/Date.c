#include "Date.h"

//����PID����
PID_Date_t Dynamic[30];
/**************************************************************************
�������ܣ�PID���ݳ�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/

void PID_Date_Init()
{
/**************λ��ʽ**************/
	/*���ٶ�PID�����Ʋ�����ʼ��*/
	Angle_speed_loop.Kp=1.92;
	Angle_speed_loop.Ki=0.6;
	Angle_speed_loop.Target_Date=0;
	Angle_speed_loop.Now_Date=0;
	
	/*�Ƕ�PID�����Ʋ�����ʼ��*/
	Angle_loop.Kp=-200;
	Angle_loop.Kd=-0.35;
	Angle_loop.Target_Date=0;
	Angle_loop.Now_Date=0;
	
	/*�Ƕ�PID�����Ʋ�����ʼ��*/
	Turn_loop.Kp=20;
	Turn_loop.Target_Date=0;
	
/**************����ʽ**************/
	/*�ٶ�PID�����Ʋ�����ʼ��*/				
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
		
		//PID��������
	}
}
