#include "sys.h"

typedef void (*FSM_Fuction_t) (void);//״̬�����ܺ���

typedef enum{
	Holding_State=0,//����״̬
	Upright_State=1,//ֱ��״̬
	Normal_Running_State=2
}FSM_State;

typedef struct{
	
	FSM_State State;
	uint8_t State_change;
	FSM_Fuction_t FSM_Function[3];
	
}FSM_t;

extern FSM_t Active_FSM;
/**********�ܵ��Ⱥ�������*********/
void FSM_Schedule(void);//���ж����л�״̬

/**********��ʼ������*************/
void TIM1_Config(u16 arr,u16 psc);
void TIM3_Config(u16 arr,u16 psc);

void FSM_Init(void);//״̬��״̬���ܺ���ע��

/**********״̬��״̬���ܺ�����*******/
void FSM_Function0(void);
void FSM_Function1(void);
void FSM_Function2(void);