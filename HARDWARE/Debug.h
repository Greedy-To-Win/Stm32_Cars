#include "sys.h"


/*******显示调试，功能选择**/
#define DEBUG_OPENMV 0

#define DEBUG_Angle_speed 1
#define DEBUG_standing 	 0
#define DEBUG_speed		 0
#define DEBUG_turn		 0
#define DEBUG_bluetooth  0

/*********串口**********/
#define Fput_to_openmv    1
#define Fput_to_bluetooth 0

#define USE_OPENMV 		0
#define USE_Vofa		0 //这里两个只能开一个.不然由于变量定义会报错




void Debug_Display(void);	//开机调试数据显示
void Init_Display(void);	//待机初始化显示