#include "sys.h"

typedef void (*FSM_Fuction_t) (void);//状态机功能函数

typedef enum{
	Holding_State=0,//待机状态
	Upright_State=1,//直立状态
	Normal_Running_State=2
}FSM_State;

typedef struct{
	
	FSM_State State;
	uint8_t State_change;
	FSM_Fuction_t FSM_Function[3];
	
}FSM_t;

extern FSM_t Active_FSM;
/**********总调度函数函数*********/
void FSM_Schedule(void);//在中断中切换状态

/**********初始化函数*************/
void TIM1_Config(u16 arr,u16 psc);
void TIM3_Config(u16 arr,u16 psc);

void FSM_Init(void);//状态机状态功能函数注册

/**********状态机状态功能函数表*******/
void FSM_Function0(void);
void FSM_Function1(void);
void FSM_Function2(void);