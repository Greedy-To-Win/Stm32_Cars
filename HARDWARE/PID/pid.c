#include "pid.h"
#include "mpu6050.h"
/******PID�����������ʼ��***********/
Position_PID_t 	Angle_loop;
Position_PID_t 	Angle_speed_loop;
Position_PID_t 	Turn_loop;
Inc_PID_t 		Motor_Velocity_loop; 
Exper_Inc_Pid_TCB_t	 Exper_Motor_Velocity_TCB;
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
	
//	if(Inc_PID->Error > Inc_PID->Error_Max)		//���̫���ⲿ����
//		Inc_PID->Out=Max_Output;
//	else if(Inc_PID->Error < Inc_PID->Error_Min)//���̫���ⲿ����
//		Inc_PID->Out=-Max_Output;
	if(Inc_PID->Error > Inc_PID->Error_Max||Inc_PID->Error < Inc_PID->Error_Min)
	 {
		 if(Inc_PID->Error*Inc_PID->Kp>0)
			Inc_PID->Out=Max_Output;
		 else
			Inc_PID->Out=-Max_Output; 
	 }
	else	//�������	
	{
		Inc_PID->Out_Inc =Inc_PID->Kp*(Inc_PID->Error-Inc_PID->Last_Error)+(Inc_PID->Ki*Inc_PID->Error);//����ʽPI������ 

		if(Inc_PID->Out_Inc>2||Inc_PID->Out_Inc<-2)
		{
		/************************/
		 Inc_PID->Out+=Inc_PID->Out_Inc;//�����ۼ�
		/*************************/
		}
		/*******����޷�*********/
		if(Inc_PID->Out>Max_Output)
			Inc_PID->Out=Max_Output;
			
		if(Inc_PID->Out<-Max_Output)
			Inc_PID->Out=-Max_Output;
	}

/****������һ��ƫ�� ****/
	Inc_PID->Last_Error=Inc_PID->Error;
}	

/**************************************************************************
�������ܣ�ר������ʽ������
��ڲ���������ʽPID����ָ�룬����ʽר�ҿ��ƿ�ָ��
����  ֵ����
***************************************************************************/

void Inc_PID_Control_Expert(Inc_PID_t* Inc_PID,Exper_Inc_Pid_TCB_t* Exper_Motor_Velocity_TCB)
{
//	//�������仯
//	Exper_Motor_Velocity_TCB->inc_iError=Inc_PID->Error - Inc_PID->Last_Error;
////	if(Inc_PID->Error<0)
////		Inc_PID->Error=-Inc_PID->Error; //����������Ϊ����ֵ
//	// Сƫ�� 
//	if (Inc_PID->Error < Exper_Motor_Velocity_TCB->Error_abs_Min) 
//		{
//			//��С���, ��С����
//			Inc_PID->Out_Inc = Exper_Motor_Velocity_TCB->control_parameter * Inc_PID->Out_Inc;
//		} 
//		
//	// ��ƫ��
//	else if(Inc_PID->Error > Exper_Motor_Velocity_TCB->Error_abs_Max) 
//		{
//			// ����BangBang-PID
//			Inc_PID->Out_Inc= (Inc_PID->Error > 0) ? (6999) : (-6999);
//		}
//	// ����ƫ��
//	else {
//			// ƫ���ڳ���ƫ�����ֵ����ķ���仯(ƫ��Խ��Խ��), ����ƫ��һֱΪĳһ�̶�ֵ
//			if ((Inc_PID->Error * Exper_Motor_Velocity_TCB->inc_iError > 0 && Error_Inc_PID->Last_Error* Exper_Motor_Velocity_TCB->inc_Error1 > 0) || (Inc_PID->Error - Inc_PID->Last_Error) == 0) 
//				{
//				
//				if (Inc_PID->Error > Exper_Motor_Velocity_TCB->Error_abs_Mid) 
//					{
//						// ������ʵʩ��ǿ�Ŀ�������
//						Inc_PID->Out_Inc = (1.2+Exper_Motor_Velocity_TCB->control_parameter )* Inc_PID->Out_Inc;
//					} 
//				else 
//					{
//						// ����ƫ�����ֵ�������Ǻܴ�
//						Inc_PID->Out_Inc =Inc_PID->Out_Inc + 0;
//					}
//				}
//			// ƫ��ľ���ֵ���С�ķ���仯�������Ѿ��ﵽƽ��״̬
//			else if ((Inc_PID->Error * Exper_Motor_Velocity_TCB->inc_iError  < 0 )&&( Exper_Motor_Velocity_TCB->inc_iError * Exper_Motor_Velocity_TCB->inc_Error1 > 0) ||( Inc_PID->Error  == 0 && Inc_PID->Last_Error == 0)) 
//				{
//					
//					// ��ʱ���Ա��ֿ������������
//				} 
//			else if (pid->errNow * (pid->errNow - pid->errLast) < 0 && ((pid->errNow - pid->errLast) * (pid->errLast - pid->errLastLast) < 0)) 
//				{
//					// ƫ��ڼ�ֵ����״̬
//					if (pid->errABS > midErr) 
//						{
//						pid->outPut = onlineK * pid->outPut;
//					} else {
//						// ����ƫ�����ֵ�������Ǻܴ�
//						pid->outPut = pid->outPut + 0;
//					}
//				
//				} 
//			else 
//			{
//				// �������
//				pid->outPut = pid->outPut + 0;
//			}
//		}

//	//�������
//	Exper_Motor_Velocity_TCB->Error1=Inc_PID->Error;
//	Exper_Motor_Velocity_TCB->Error2=Exper_Motor_Velocity_TCB->Error1;

//	//�����������
//	Exper_Motor_Velocity_TCB->inc_Error1=Exper_Motor_Velocity_TCB->inc_iError;
	
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
	//������
	 if(Position_PID->Error>Position_PID->Error_Max||Position_PID->Error < Position_PID->Error_Min)
	 {
		 if(Position_PID->Error*Position_PID->Kp>0)
			Position_PID->Out=Max_Output;
		 else
			Position_PID->Out=-Max_Output; 
	 }
	 else{
				 
			//�����������������
			if(Position_PID->Error > -Position_PID->Error_temp && Position_PID->Error < Position_PID->Error_temp)
			{
				if(Position_PID->Error_sum>Max_i_date)
					Position_PID->Error_sum=Max_i_date;
					
				if(Position_PID->Error_sum<-Max_i_date)
					Position_PID->Error_sum=-Max_i_date;
			}
			
			else 
				Position_PID->Error_sum=0;//�������û���

			
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
			}
			
			
			/*******����޷�*********/

			if(Position_PID->Out>Max_Output)
				Position_PID->Out=Max_Output;
				
			if(Position_PID->Out<-Max_Output)
				Position_PID->Out=-Max_Output;
			//������һ��ƫ��
			Position_PID->Last_Error=Position_PID->Error;
		}
}

void Motor_Velocity_Set(int16_t speed)
{
	Motor_Velocity_loop.Target_Date=speed;
}


