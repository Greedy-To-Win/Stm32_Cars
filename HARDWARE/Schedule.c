#include "Schedule.h"
#include "pid.h"
#include "motor.h"
#include "oled.h"
#include "encoder.h"
#include "mpu6050.h"
#include "open_mv_usart.h"
FSM_t Active_FSM;
Mpu6050_Date mpu6050_date;				//mpu6050ԭʼ���ݺ�dmp�����ŷ���ǣ���pid���㺯���л�ȡ

void FSM_Function0(void)//����
{
	Init_Display();//������ʼ����ʾ
}
void FSM_Function1(void)//����ֱ��
{
	Debug_Display();//����������ʾ
}
void FSM_Function2(void)//������ʻ��������Ԫ��
{
	//PIDת��
}
void FSM_Init(void)
{
	Active_FSM.State=Holding_State;
	Active_FSM.State_change=0;
	Active_FSM.FSM_Function[Holding_State]=FSM_Function0;
	Active_FSM.FSM_Function[Upright_State]=FSM_Function1;
	Active_FSM.FSM_Function[Normal_Running_State]=FSM_Function2;
	
}



void FSM_Schedule(void)//���ж����л�״̬
{
	if(1==Active_FSM.State_change)
	{
		OLED_Refresh();
		Active_FSM.State_change=0;
	}
	if(Upright_State==Active_FSM.State)//����ֱ��
	{
		Active_FSM.FSM_Function[Upright_State]();
	}
	else if(Normal_Running_State==Active_FSM.State)//������ʻ
	{
		Active_FSM.FSM_Function[Normal_Running_State]();
	}
	else//����
	{

		Active_FSM.FSM_Function[Holding_State]();
	}
}


void TIM3_Config(u16 arr,u16 psc)
{

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
    TIM_ClearITPendingBit(TIM3,TIM_IT_Update);TIM_ClearFlag(TIM3,TIM_FLAG_Update);//??????
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;

    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		TIM_Cmd(TIM3,ENABLE);
}

/**************************************************************************
�������ܣ�TIM3�жϺ���������PID
��ڲ����� ��
����  ֵ����
**************************************************************************/

void TIM3_IRQHandler(void)//2ms//500HZ
{

	if(TIM_GetITStatus(TIM3,TIM_IT_Update)!=RESET)
	{	
//		if( 1 == vofa_packet.contrl_flag )//���ж�����λ
//		{
//			vofa_packet.contrl_flag=0;
//			Get_vofa_date_to_pid(&vofa_packet);
//		}
//		
		if(mpu_dmp_get_data(&mpu6050_date.pitch,&mpu6050_date.roll,&mpu6050_date.yaw)==0)   //��ȡŷ����
		{ 
			MPU_Get_Gyroscope(&mpu6050_date.gyrox,&mpu6050_date.gyroy,&mpu6050_date.gyroz);	//�õ�����������
			//===��ȡ��������ֵ
			Blance_motor.Velocity_Left=-Read_Encoder(2);
			Blance_motor.Velocity_Right=Read_Encoder(4);	 
/*****************************������ʵ��ֵ��ȡ****************************************/		
			Motor_Velocity_loop.Now_Date=Blance_motor.Velocity_Left+Blance_motor.Velocity_Right;
			Angle_loop.Now_Date=mpu6050_date.roll;
			if(Angle_loop.Now_Date>=45||Angle_loop.Now_Date<=-45)
			{
				Blance_motor.Motor_state=Motor_Error;
				Motor_Error_Function();//���ͣת
			}
			else
			{
				Blance_motor.Motor_state=Motor_Normal;
			}
			Angle_loop.Kd_date=mpu6050_date.gyrox;
			Angle_speed_loop.Now_Date=mpu6050_date.gyrox+34;
			Turn_loop.Now_Date=mpu6050_date.yaw;
		}
/************************************************************************************/
		if(Active_FSM.State!=Holding_State)
		{
			TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
			//��λ��������
			printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",\
			(float)Motor_Velocity_loop.Target_Date/2,Motor_Velocity_loop.Out,\
			(float)Blance_motor.Velocity_Left,(float)Blance_motor.Velocity_Right,Angle_loop.Out,Angle_speed_loop.Out,(float)mpu6050_date.gyrox,mpu6050_date.roll,Angle_loop.Now_Date);//9
			
			//���ڻ�,��������1999���������4999��΢������Ҫ����
			//�ǶȻ������Ϊ����ֵ
			/****************���ٶ��ڻ�****************/
			Angle_speed_loop.timer++;
			Angle_speed_loop.Target_Date = Angle_loop.Out;
			Position_PID_Control(&Angle_speed_loop,\
									6999,1999,0);
			//ֻ����kp��ת��
			Turn_PID_Control(&Turn_loop,3999,0);
				
	/****************PWM�������***************/	
			Blance_motor.Moto1_pwm_out = Angle_speed_loop.Out - Turn_loop.Out;
			Blance_motor.Moto2_pwm_out = Angle_speed_loop.Out + Turn_loop.Out;
			//��PWM�����޷�
			Xianfu_Pwm();				
			//����PWM
			Set_Pwm(Blance_motor.Moto1_pwm_out,Blance_motor.Moto2_pwm_out);		



	/****************�Ƕ��л�****************/
			Angle_loop.timer++;//10ms
			if(Angle_loop.timer>=5)
			{
				//�л�,΢����ֱ��ʹ�ý��ٶ�
				Angle_loop.Target_Date=Motor_Velocity_loop.Out;
				Position_PID_Control(&Angle_loop,4999,0,1);
				Angle_loop.timer=0;
			}
	/**************����ٶ��⻷**************/
			Motor_Velocity_loop.timer++;
			if(Motor_Velocity_loop.timer>=40)
			{
				//�⻷
				Motor_Velocity_Set(30);//�����ٶ�
				//����ٶȻ����ƣ�������Ϊ199
				Inc_PID_Control(&Motor_Velocity_loop,199);
				Motor_Velocity_loop.timer=0;
			}
		}
//		/***************������0**************/
//			if(Angle_speed_loop.timer>=30&&Angle_speed_loop.Error_sum>=3999)
//			{
//				Angle_speed_loop.timer=0;
//				Angle_speed_loop.Error_sum=0;
//			}
	}
}
