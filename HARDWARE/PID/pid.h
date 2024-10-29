#ifndef __PID_H__
#define __PID_H__
#include "sys.h"
//λ��ʽPID�ṹ��
typedef struct 
{
	float Kp;
	float Ki;
	float Kd;
	float Out;
	float Now_Date;
	float Target_Date;
	float Error;
	float Last_Error;
	float Error_sum;
	float Kd_date;//�ǶȻ���΢��Ϊ���ٶȣ�����ҪError-LastError
	float		 Error_Max;//ƫ�����ֵ
	float		 Error_temp;//ƫ�����ֵ�н飬//�����ж��Ƿ����
	float		 Error_Min;//ƫ����Сֵ
	uint8_t timer;
}Position_PID_t;
//����ʽPID�ṹ��
typedef struct 
{
	float Kp;
	float Ki;
	float Kd;
	float Out;
	float Out_Inc;
	float Now_Date;
	float Target_Date;
	float Error;
	float Last_Error;
	float		 Error_Max;//ƫ�����ֵ
	float		 Error_temp;//ƫ�����ֵ�н飬//�����ж��Ƿ����
	float		 Error_Min;//ƫ����Сֵ
	uint8_t timer;
}Inc_PID_t;

typedef struct
{
	
	volatile float      Error1; // Error[n-1]
	volatile float      Error2; // Error[n-2]
	volatile float		 Error_sum;//����ֵ
	volatile float		 iIncpid;//PID���ֵ
	volatile float    inc_iError;//Error[n]����
	volatile float    inc_Error1;//Error[n-1]����
	volatile float		 Error_abs_Max;//ƫ�����ֵ���ֵ
	volatile float		 Error_abs_Mid;//ƫ�����ֵ����ֵ
	volatile float		 Error_abs_Min;//ƫ�����ֵ��Сֵ
	float control_parameter;//������Ʋ���
} Exper_Inc_Pid_TCB_t;


extern Exper_Inc_Pid_TCB_t	 Exper_Motor_Velocity_TCB;//����ʽר�ҿ��ƿ�
extern Position_PID_t 	 Angle_loop;
extern Position_PID_t 	 Angle_speed_loop;
extern Position_PID_t 	 Turn_loop;
extern Inc_PID_t 		 Motor_Velocity_loop;



void Inc_PID_Control(Inc_PID_t* Inc_PID,uint16_t Max_Output);
void Position_PID_Control(Position_PID_t* Position_PID,uint16_t Max_Output,uint16_t Max_i_date,uint8_t Using_kd_date);
void Turn_PID_Control(Position_PID_t* Turn_PID,uint16_t Max_Output,uint8_t Using_kd_date);

//����ר�ҿ��ƿ������ʽר��PID������
void Inc_PID_Control_Expert(Inc_PID_t* Inc_PID,Exper_Inc_Pid_TCB_t* Exper_Motor_Velocity_TCB);

void Motor_Velocity_Set(int16_t speed);
#endif
