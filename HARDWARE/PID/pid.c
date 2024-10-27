#include "pid.h"
#include "mpu6050.h"
/******PID�����������ʼ��***********/
Position_PID_t 	Angle_loop;
Position_PID_t 	Angle_speed_loop;
Position_PID_t 	Turn_loop;
Inc_PID_t 		Motor_Velocity_loop; 

/**************************************************************************
�������ܣ�ת�򻷿�����
��ڲ�����PID����ָ�룬
			������ֵ,
			΢�����ѡ��Ϊ1��ֱ��ʹ�ö�ȡ�����ݣ�Ϊ0����Ҫǰ�����������м���
����  ֵ����
***************************************************************************/				
void Turn_PID_Control(Position_PID_t* Turn_PID,uint16_t Max_Output,uint8_t Using_kd_date)
{

		
	if(Turn_PID->Target_Date <= -180&& Turn_PID->Now_Date  >0)
		Turn_PID->Error = (Turn_PID->Now_Date -(Turn_PID->Target_Date+360))*10;	
	else if(Turn_PID->Target_Date<= -360&&Turn_PID->Now_Date <0)
		Turn_PID->Error = (Turn_PID->Now_Date -(Turn_PID->Target_Date+360))*10;	
	else if(Turn_PID->Target_Date >= 180&&Turn_PID->Now_Date <0)
		 Turn_PID->Error = (Turn_PID->Now_Date -(Turn_PID->Target_Date-360))*10;
	else if(Turn_PID->Target_Date>= 360&&Turn_PID->Now_Date >0)
		Turn_PID->Error = (Turn_PID->Now_Date -(Turn_PID->Target_Date-360))*10;
	else
		Turn_PID->Error = (Turn_PID->Now_Date -Turn_PID->Target_Date)*10;	
	if(Turn_PID->Error>-0.15&&Turn_PID->Error<0.15)
		Turn_PID->Error=0;
	//ת�����
	 Turn_PID->Out = Turn_PID->Kp*Turn_PID->Error; 
	
				/*����ת���ٶȵĵ����޷�*/
	if(Turn_PID->Out>=1500)	Turn_PID->Out =1500;
	else if(Turn_PID->Out<=-1500)Turn_PID->Out =-1500;
			
	
}

/**************************************************************************
�������ܣ���ͨ����ʽ������
��ڲ���������ʽPID����ָ�룬������ֵ
����  ֵ����
***************************************************************************/

void Inc_PID_Control(Inc_PID_t* Inc_PID,uint16_t Max_Output)
{
	//������
	Inc_PID->Error=Inc_PID->Now_Date - Inc_PID->Target_Date;
/**************�˲�***************/
	Inc_PID->Error*=0.8;
	if(Inc_PID->Error>-1.5&&Inc_PID->Error<1.5)
	{
		Inc_PID->Error=0;
	}
/********************************/
	//��������
	Inc_PID->Out_Inc =Inc_PID->Kp*(Inc_PID->Error-Inc_PID->Last_Error)+(Inc_PID->Ki*Inc_PID->Error);//����ʽPI������ 
	//������һ��ƫ�� 
	Inc_PID->Last_Error=Inc_PID->Error;
	if(Inc_PID->Out_Inc>2||Inc_PID->Out_Inc<-2)
	{
		Inc_PID->Out+=Inc_PID->Out_Inc;
	}
/*******����޷�*********/
	if(Inc_PID->Out>Max_Output)
		Inc_PID->Out=Max_Output;
		
	if(Inc_PID->Out<-Max_Output)
		Inc_PID->Out=-Max_Output;
}

/**************************************************************************
�������ܣ���ͨλ��ʽ������
��ڲ�����λ��ʽPID����ָ�룬
			������ֵ,
			�������ֵ��
			΢�����ѡ��Ϊ1��ֱ��ʹ�ö�ȡ�����ݣ�Ϊ0����Ҫǰ�����������м���
����  ֵ����
***************************************************************************/

void Position_PID_Control(Position_PID_t* Position_PID,uint16_t Max_Output,uint16_t Max_i_date,uint8_t Using_kd_date)
{
	//������
	Position_PID->Error=Position_PID->Now_Date-Position_PID->Target_Date;
	Position_PID->Error*=0.8;
	Position_PID->Error_sum+=Position_PID->Error;
	//�����޷�
	if(Position_PID->Error_sum>Max_i_date)
		Position_PID->Error_sum=Max_i_date;
		
	if(Position_PID->Error_sum<-Max_i_date)
		Position_PID->Error_sum=-Max_i_date;
	
	if(1==Using_kd_date)//��ֱ�Ӷ���΢����
	{
		Position_PID->Out=Position_PID->Kp*Position_PID->Error\
					  +Position_PID->Ki*Position_PID->Error_sum\
					  +Position_PID->Kd*Position_PID->Kd_date;
	}
	else//����ֱ�Ӷ���΢�������������������
	{
		Position_PID->Out=Position_PID->Kp*Position_PID->Error\
					  +Position_PID->Ki*Position_PID->Error_sum\
			+Position_PID->Kd*(Position_PID->Error-Position_PID->Last_Error);
	//������һ��ƫ�� 	
	Position_PID->Last_Error=Position_PID->Error;
	}
	
	
	/*******����޷�*********/
	if(Position_PID->Out>Max_Output)
		Position_PID->Out=Max_Output;
		
	if(Position_PID->Out<-Max_Output)
		Position_PID->Out=-Max_Output;
}

void Motor_Velocity_Set(int16_t speed)
{
	Motor_Velocity_loop.Target_Date=speed;
}


