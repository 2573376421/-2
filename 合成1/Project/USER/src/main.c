#include "headfile.h"


//board.h�ļ���FOSC��ֵ����Ϊ0,������Զ�ʶ��ϵͳƵ��

/*board.h�ļ���FOSC��ֵ���ò�Ϊ0����ϵͳƵ��ΪFOSC��ֵ��
��ʹ��stc-isp�������س����ʱ����Ҫ��IRCƵ������ΪFOSC��ֵ*/

/*��board_init��,�Ѿ���P54��������Ϊ��λ��
�����Ҫʹ��P54����,������board.c�ļ��е�board_init()������ɾ��SET_P54_RESRT����*/

#include "headfile.h"

void main()
{
//	float v;
	Init();
    while(1)
	{

//		v = adc_voltage();
//		oled_p6x8str(0,0,"v");
//		oled_p6x8str(0,1,"ANGLE");
//		oled_p6x8str(0,2,"AD_Err");
//		oled_p6x8str(0,3,"AD1");
//		oled_p6x8str(0,4,"AD2");
//		oled_p6x8str(0,5,"AD3");
//		oled_p6x8str(0,6,"AD4");
//		oled_p6x8str(0,7,"check");
//		
		oled_printf_float(50,0,Angle_lv,4,2);
//		oled_printf_float(50,1,count,4,2);
//		oled_printf_float(50,2,MOTOR1,4,2);
//		oled_int16(50,3,Filter_AD1);
//		oled_int16(50,4,Filter_AD2);
//		oled_int16(50,5,Filter_AD3);
//		oled_int16(50,6,Filter_AD4);
//		oled_printf_float(50,7,v,4,2);
//		bluetooth_osc(Angle_lv,MOTOR1,MOTOR2,1);
		
    }
}

