#include "sys.h"

#include "delay.h"	
#include "key.h"   
#include "oled.h"
#include "Debug.h"
#include "Date.h"

#include "mpu6050.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"

#include "bluetooth.h"
#include "usart.h"	 //�����ض�����usart2��openmv��
#include "open_mv_usart.h"

#include "Schedule.h"//�������

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 







/*
-------------------------------------IO�ڶ�Ӧ���ܱ�-------------------------------------------------------
  ���������               B12->AIN2   B13->AIN1    B14->BIN1    B15->BIN2		
    PWM   ��               A11->PWMA                A8->PWMB					
    OLED  :                 B8 ->SCL                B9 ->SDA					
  MUP6050 :                 B4 ->SCL                B3 ->SDA					
�����������              B6 B7 һ��                A0 A1һ��				

Openmv	:(usart2)			RXD ->A3               TXD->A2
����    :                   RXD ->A10              TXD->A9(Rx���ⲿTx)
����    :               	A4			A5			A6			A7	                    

*/
/*
--------------------------------------�����ж����ȼ�--------------------------------------------------------
�ж����ȼ����ã�			��ռ���ȼ�				�����ȼ�
TIM3��PID���Ƽ��㷴����		0						0
TIM1(������					0						1
Openmv(usart2)				1						0
Bluetooth(usart1)			1						1
Usart3						1						3
*/




extern uint8_t* BUF_P1;								//������������һ�ֽڵ�ַ
extern Bluetooth_packet_t bluetooth_packet;			//�������ݰ�
extern Openmv_usart_packet_t openmv_tracking_packet;//openmv���ݰ�


/**************************************************************************
�������ܣ�������
��ڲ�������
����  ֵ����
**************************************************************************/

int main(void)
 { 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   
	 delay_init();	 //��ʱ������ʼ��	  
	 OLED_Init();                    //OLED��ʼ��	
	 KEY_Init();
	 MPU_Init();//�д���ȶ���з���ֵ��������200hz//��ʼ��MPU6050
	 OLED_Refresh();		      
	
	 openmv_usart_init(115200);	 //PID��ʼ��
#if DEBUG_OPENMV 
	 printf("����OK\r\n");                                       
	 Usart_SendString( OpenMV_USART_USARTx,"����һ�������жϽ��ջ���ʵ��\n");
#endif
	while(mpu_dmp_init())
	{
		OLED_ShowString(2,1,"mpu6050.....");
		delay_ms(1500);
	} 								//����DMP��ʼ��
	MiniBalance_PWM_Init(7199,0);   //��ʼ��PWM 10KHZ������������� ,PWM���7200
	TIM3_Config(1999,71);//��ֹ�ж�Ӱ���ʼ���������ж�ʱ���жϵĶ���
	Encoder_Init_TIM2();            //=====�������ӿ�
    Encoder_Init_TIM4();            //=====��ʼ��������2
	OLED_ShowString(3,1,"finsh");
	delay_ms(500);
	OLED_Refresh();
/*************************************************************/	
	FSM_Init();//���� ,״̬����ʼ��
	Motor_Date_Init();//���״̬��������ֵ��ʼ��
	PID_Date_Init();
 	while(1)
	{
		//״̬���л��������д�����ֱ���ɰ����л�
		  FSM_Schedule();

	} 	
}

