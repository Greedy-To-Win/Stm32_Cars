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
#include "usart.h"	 //串口重定向是usart2（openmv）
#include "open_mv_usart.h"

#include "Schedule.h"//程序调度

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 







/*
-------------------------------------IO口对应功能表-------------------------------------------------------
  电机驱动：               B12->AIN2   B13->AIN1    B14->BIN1    B15->BIN2		
    PWM   ：               A11->PWMA                A8->PWMB					
    OLED  :                 B8 ->SCL                B9 ->SDA					
  MUP6050 :                 B4 ->SCL                B3 ->SDA					
电机编码器：              B6 B7 一组                A0 A1一组				

Openmv	:(usart2)			RXD ->A3               TXD->A2
蓝牙    :                   RXD ->A10              TXD->A9(Rx接外部Tx)
按键    :               	A4			A5			A6			A7	                    

*/
/*
--------------------------------------程序中断优先级--------------------------------------------------------
中断优先级配置：			抢占优先级				子优先级
TIM3（PID控制计算反馈）		0						0
TIM1(采样）					0						1
Openmv(usart2)				1						0
Bluetooth(usart1)			1						1
Usart3						1						3
*/




extern uint8_t* BUF_P1;								//蓝牙缓冲区第一字节地址
extern Bluetooth_packet_t bluetooth_packet;			//蓝牙数据包
extern Openmv_usart_packet_t openmv_tracking_packet;//openmv数据包


/**************************************************************************
函数功能：主函数
入口参数：无
返回  值：无
**************************************************************************/

int main(void)
 { 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
   
	 delay_init();	 //延时函数初始化	  
	 OLED_Init();                    //OLED初始化	
	 KEY_Init();
	 MPU_Init();//有待商榷（有返回值）采样率200hz//初始化MPU6050
	 OLED_Refresh();		      
	
	 openmv_usart_init(115200);	 //PID初始化
#if DEBUG_OPENMV 
	 printf("发送OK\r\n");                                       
	 Usart_SendString( OpenMV_USART_USARTx,"这是一个串口中断接收回显实验\n");
#endif
	while(mpu_dmp_init())
	{
		OLED_ShowString(2,1,"mpu6050.....");
		delay_ms(1500);
	} 								//进行DMP初始化
	MiniBalance_PWM_Init(7199,0);   //初始化PWM 10KHZ，用于驱动电机 ,PWM最多7200
	TIM3_Config(1999,71);//防止中断影响初始化，最后进行定时器中断的定义
	Encoder_Init_TIM2();            //=====编码器接口
    Encoder_Init_TIM4();            //=====初始化编码器2
	OLED_ShowString(3,1,"finsh");
	delay_ms(500);
	OLED_Refresh();
/*************************************************************/	
	FSM_Init();//待机 ,状态机初始化
	Motor_Date_Init();//电机状态、输出最大值初始化
	PID_Date_Init();
 	while(1)
	{
		//状态机切换任务，其中待机到直立由按键切换
		  FSM_Schedule();

	} 	
}

