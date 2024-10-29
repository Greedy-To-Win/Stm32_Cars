#include "key.h"
#include "delay.h"
#include "pid.h"
#include "Debug.h"
#include "Schedule.h"
/**************************************************************************
�������ܣ�������ʼ������
��ڲ�������
����  ֵ����
**************************************************************************/	 
void KEY_Init(void)
{
	
	
	GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    /*****����ʱ��*************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO  , ENABLE);
    
    /******����GPIO��**********/
 	//GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//�ر�jtag��ʹ��SWD��������SWDģʽ����
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//PA5.PA4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //���ó���������
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /*******����AFIO �ж�����ѡ��********/
	//GPIO_EXTILineConfig(GPIO_PortSourceGPIOB , GPIO_PinSource14);
    
    /*****����EXTI********/

    EXTI_InitStructure.EXTI_Line = EXTI_Line4|EXTI_Line5| EXTI_Line6|EXTI_Line7;                     //ѡ���ж���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                       //ʹ��
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;             //�ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;         //ѡ���½��ش���
    EXTI_Init(&EXTI_InitStructure);
    
    /*****����NVIC*****/

    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;                       // ʹ�����ڵ��ⲿ�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                             //ʹ���ⲿ�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                   //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //��Ӧ���ȼ�
    NVIC_Init(&NVIC_InitStructure);
    
	NVIC_InitStructure.NVIC_IRQChannel =EXTI9_5_IRQn ;                       // ʹ�����ڵ��ⲿ�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                             //ʹ���ⲿ�ж�ͨ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                   //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //��Ӧ���ȼ�
    NVIC_Init(&NVIC_InitStructure);
}





void  EXTI4_IRQHandler(void)     //�ⲿ�жϺ���
{
    if(EXTI_GetITStatus(EXTI_Line4) == SET)    //�ж��Ƿ�ͨ��4�����ж�
    {
		if(Holding_State==Active_FSM.State)//�������״̬�������л�������ֱ��״̬
		{
			Active_FSM.State_change=1;
			Active_FSM.State=Upright_State;//����ֱ��
		}
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0&&Active_FSM.State!=Holding_State)         //���4�Ž�Ϊ�͵�ƽ
		{
#if DEBUG_Angle_speed
			Angle_speed_loop.Kp+=0.01;
#endif
#if DEBUG_standing			
				Angle_loop.Kp+=10;
		
#endif	
#if DEBUG_speed
			Motor_Velocity_loop.Ki+=2;
#endif			
#if DEBUG_turn			
			Turn_loop.Kp+=1;
#endif						
	

        EXTI_ClearITPendingBit(EXTI_Line4);        //����ж�����
		}
	}
}
	
void EXTI9_5_IRQHandler	(void)
{	
/**************************����A5*******************************************/		
    if(EXTI_GetITStatus(EXTI_Line5) == SET)    //�ж��Ƿ�ͨ��5�����ж�
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0&&Active_FSM.State!=Holding_State)         //���5�Ž�Ϊ�͵�ƽ
		{
#if DEBUG_Angle_speed
			Angle_speed_loop.Kp-=0.01;
#endif			
#if DEBUG_standing	

				Angle_loop.Kp-=10;
#endif
#if DEBUG_speed			
			Motor_Velocity_loop.Ki-=2;
#endif			
#if DEBUG_turn			
			Turn_loop.Kp-=1;
#endif			
		
		}
        EXTI_ClearITPendingBit(EXTI_Line5);        //����ж�����
    }

/**************************����A6*******************************************/		
	 if(EXTI_GetITStatus(EXTI_Line6) == SET)    //�ж��Ƿ�ͨ��5�����ж�
    {
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6) == 0&&Active_FSM.State!=Holding_State)         //���5�Ž�Ϊ�͵�ƽ
		{
#if DEBUG_Angle_speed
			Angle_speed_loop.Ki+=0.001;
#endif					
#if DEBUG_standing	
				Angle_loop.Kd+=0.01;
#endif
#if DEBUG_speed			
			    Motor_Velocity_loop.Kp+=0.1;
#endif			


        EXTI_ClearITPendingBit(EXTI_Line6);        //����ж�����
		}
	}
/**************************����A7*******************************************/	
	
	if(EXTI_GetITStatus(EXTI_Line7) == SET)    //�ж��Ƿ�ͨ��5�����ж�
	{
        if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == 0&&Active_FSM.State!=Holding_State)         //���5�Ž�Ϊ�͵�ƽ
		{
#if DEBUG_Angle_speed
			Angle_speed_loop.Ki-=0.001;
#endif	
			
#if DEBUG_standing	
				Angle_loop.Kd-=0.01;
			
#endif
#if DEBUG_speed		
			Motor_Velocity_loop.Kp-=0.1;			
#endif			

		}
        EXTI_ClearITPendingBit(EXTI_Line7);        //����ж�����
    }
}
