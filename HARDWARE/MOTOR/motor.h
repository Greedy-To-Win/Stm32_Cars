#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 
#define PWMA   TIM1->CCR4  //PA11
#define AIN2   PBout(12)
#define AIN1   PBout(13)
#define BIN1   PBout(14)
#define BIN2   PBout(15)
#define PWMB   TIM1->CCR1  //PA8


/****�������״̬����****/	
typedef enum
{
	Motor_Error=0,
	Motor_Normal=1
}Motor_state_TypeDef;
/****�����******/
typedef struct{
	int Moto1_pwm_out;
	int Moto2_pwm_out;
	int Velocity_Left;
	int Velocity_Right;//���ҵ���ٶȣ���encoder.c��tim3�ж�ִ��

	Motor_state_TypeDef Motor_state;
	int Motor_out_max;//������pwm�������޷�
}Motor_t;
	
extern Motor_t Blance_motor;


void MiniBalance_PWM_Init(u16 arr,u16 psc);
void MiniBalance_Motor_Init(void);
int myabs(int a);

//���õ��pwmֵ��ͨ��������������ת
void Set_Pwm(int moto1,int moto2);

void Xianfu_Pwm(void);
void Motor_Error_Function(void);

#endif
