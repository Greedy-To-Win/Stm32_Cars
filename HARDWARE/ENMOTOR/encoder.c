#include "encoder.h"
#include "stm32f10x_gpio.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "motor.h"
#include "pid.h"
#include "open_mv_usart.h"

/**************************************************************************
函数功能：把TIM2初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
		TIM_ICInitTypeDef TIM_ICInitStructure;  
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能定时器4的时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PB端口时钟
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;	//端口配置
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
		GPIO_Init(GPIOA, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB
		
		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
		TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //设定计数器自动重装值
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_ICPSC_DIV1 ;//选择时钟分频：不分频
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
		TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
		TIM_ICStructInit(&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_ICFilter = 0x06;
		TIM_ICInit(TIM2, &TIM_ICInitStructure);
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除TIM的更新标志位
		//TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		
		TIM_SetCounter(TIM2,0);
		TIM_Cmd(TIM2, ENABLE); 
}
/**************************************************************************
函数功能：把TIM4初始化为编码器接口模式
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM4(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
		TIM_ICInitTypeDef TIM_ICInitStructure;  
		GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PB端口时钟
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
		GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB
		
		TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
		TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
		TIM_TimeBaseStructure.TIM_Period = 0xffff; //设定计数器自动重装值
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_ICPSC_DIV1 ;//选择时钟分频：不分频
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
		TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
		TIM_ICStructInit(&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_ICFilter = 0x06;
		TIM_ICInit(TIM4, &TIM_ICInitStructure);
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
		//TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
		//Reset counter
		TIM_SetCounter(TIM4,0);
		TIM_Cmd(TIM4, ENABLE); 
}



/**************************************************************************
函数功能：单位时间读取编码器计数
入口参数：定时器
返回  值：计数器值
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
	int16_t value_1;
	int16_t value_2;
   switch(TIMX)
	 {
		case 2:  value_1= (uint16_t)TIM2 -> CNT; TIM2 -> CNT=0;
		 //value_2 = TIM2_Circle_Count;//获取溢出次数
		 break;
		case 4:  value_1= (uint16_t)TIM4 -> CNT;  TIM4 -> CNT=0;
		 //value_2 = TIM4_Circle_Count;//获取溢出次数
		 break;	
		default: value_1=0;
	 }

	 	
		return value_1 ;//+ value_2*65535;//计算总计数值
/*
	 使用上述代码计算编码器脉冲时遇到了以下问题，当电机反转时，计数器会向下计数，然而每次计数器都会从0开始计数，
	 所以就会直接溢出，从65535向下计数，TIM_GetCounter返回值的类型是uint16_t，也就是无符号，只有正数，
	 如果用int型接收，32767到65535的部分会变成-32767到-1，加上溢出次数乘65535就会变成很大的负值，导致出错，
	 所以value1必须用uint16_t接收，另外计算溢出次数时，如果是反转溢出次数会变成负数，所以value2必须用有符号类型int接收。

 */
}
/**************************************************************************
函数功能：TIM4中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//溢出中断
	{    
////DIR==0，判断当前计数方向，如果是向上计数表示正转，则溢出次数++		
//        if((((TIM4->CR1)>>4) & 0x01)==0)TIM4_Circle_Count++;
////DIR==1，向下计数表示反转，溢出次数--
//		else                            TIM4_Circle_Count--;		
	}				   
	TIM4->SR&=~(1<<0);//清除中断标志位 	    
}
/**************************************************************************
函数功能：TIM2中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM2->SR&0X0001)//溢出中断
	{    
////DIR==0，判断当前计数方向，如果是向上计数表示正转，则溢出次数++		
//        if((((TIM2->CR1)>>4) & 0x01)==0)TIM2_Circle_Count++;
////DIR==1，向下计数表示反转，溢出次数--
//		else                            TIM2_Circle_Count--;


	}				   
	TIM2->SR&=~(1<<0);//清除中断标志位 	    
	
}

