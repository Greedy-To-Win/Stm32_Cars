#include "sys.h"


/*******��ʾ���ԣ�����ѡ��**/
#define DEBUG_OPENMV 0

#define DEBUG_Angle_speed 1
#define DEBUG_standing 	 0
#define DEBUG_speed		 0
#define DEBUG_turn		 0
#define DEBUG_bluetooth  0

/*********����**********/
#define Fput_to_openmv    1
#define Fput_to_bluetooth 0

#define USE_OPENMV 		0
#define USE_Vofa		0 //��������ֻ�ܿ�һ��.��Ȼ���ڱ�������ᱨ��




void Debug_Display(void);	//��������������ʾ
void Init_Display(void);	//������ʼ����ʾ