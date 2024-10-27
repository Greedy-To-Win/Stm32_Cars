#include "encoder.h"
#include "stm32f10x_gpio.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "motor.h"
#include "pid.h"
#include "open_mv_usart.h"

/**************************************************************************
�������ܣ���TIM2��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM2(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
		TIM_ICInitTypeDef TIM_ICInitStructure;  
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//ʹ�ܶ�ʱ��4��ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PB�˿�ʱ��
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	//�˿�����
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
		GPIO_Init(GPIOA, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB
		
		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
		TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //�趨�������Զ���װֵ
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_ICPSC_DIV1 ;//ѡ��ʱ�ӷ�Ƶ������Ƶ
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
		TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
		TIM_ICStructInit(&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_ICFilter = 0x06;
		TIM_ICInit(TIM2, &TIM_ICInitStructure);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
		//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		
		TIM_SetCounter(TIM2,0);
		TIM_Cmd(TIM2, ENABLE); 
}
/**************************************************************************
�������ܣ���TIM4��ʼ��Ϊ�������ӿ�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Encoder_Init_TIM4(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
		TIM_ICInitTypeDef TIM_ICInitStructure;  
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//ʹ�ܶ�ʱ��4��ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//ʹ��PB�˿�ʱ��
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//�˿�����
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
		GPIO_Init(GPIOB, &GPIO_InitStructure);					      //�����趨������ʼ��GPIOB
		
		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // Ԥ��Ƶ�� 
		TIM_TimeBaseStructure.TIM_Period = 0xffff; //�趨�������Զ���װֵ
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_ICPSC_DIV1 ;//ѡ��ʱ�ӷ�Ƶ������Ƶ
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM���ϼ���  
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
		TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//ʹ�ñ�����ģʽ3
		TIM_ICStructInit(&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_ICFilter = 0x06;
		TIM_ICInit(TIM4, &TIM_ICInitStructure);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);//���TIM�ĸ��±�־λ
		//TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
		//Reset counter
		TIM_SetCounter(TIM4,0);
		TIM_Cmd(TIM4, ENABLE); 
}



/**************************************************************************
�������ܣ���λʱ���ȡ����������
��ڲ�������ʱ��
����  ֵ��������ֵ
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
	int16_t value_1;
	int16_t value_2;
   switch(TIMX)
	 {
		case 2:  value_1= (uint16_t)TIM2 -> CNT; TIM2 -> CNT=0;
		 //value_2 = TIM2_Circle_Count;//��ȡ�������
		 break;
		case 4:  value_1= (uint16_t)TIM4 -> CNT;  TIM4 -> CNT=0;
		 //value_2 = TIM4_Circle_Count;//��ȡ�������
		 break;	
		default: value_1=0;
	 }

	 	
		return value_1 ;//+ value_2*65535;//�����ܼ���ֵ
/*
	 ʹ����������������������ʱ�������������⣬�������תʱ�������������¼�����Ȼ��ÿ�μ����������0��ʼ������
	 ���Ծͻ�ֱ���������65535���¼�����TIM_GetCounter����ֵ��������uint16_t��Ҳ�����޷��ţ�ֻ��������
	 �����int�ͽ��գ�32767��65535�Ĳ��ֻ���-32767��-1���������������65535�ͻ��ɺܴ�ĸ�ֵ�����³���
	 ����value1������uint16_t���գ���������������ʱ������Ƿ�ת����������ɸ���������value2�������з�������int���ա�

 */
}
/**************************************************************************
�������ܣ�TIM4�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//����ж�
	{    
////DIR==0���жϵ�ǰ����������������ϼ�����ʾ��ת�����������++		
//        if((((TIM4->CR1)>>4) & 0x01)==0)TIM4_Circle_Count++;
////DIR==1�����¼�����ʾ��ת���������--
//		else                            TIM4_Circle_Count--;		
	}				   
	TIM4->SR&=~(1<<0);//����жϱ�־λ 	    
}
/**************************************************************************
�������ܣ�TIM2�жϷ�����
��ڲ�������
����  ֵ����
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//����ж�
	{    
////DIR==0���жϵ�ǰ����������������ϼ�����ʾ��ת�����������++		
//        if((((TIM2->CR1)>>4) & 0x01)==0)TIM2_Circle_Count++;
////DIR==1�����¼�����ʾ��ת���������--
//		else                            TIM2_Circle_Count--;


	}				   
	TIM2->SR&=~(1<<0);//����жϱ�־λ 	    
	
}

