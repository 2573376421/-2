#include "motor.h"
#include "zf_pwm.h"
#include "STC8Hxx.h"


/*�޷�����*/
void Limit(int16 *motoA,int16 *motoB)
{
	if(*motoA > PWM_MAX) *motoA=PWM_MAX;
	if(*motoA < PWM_MIN) *motoA=PWM_MIN;
	
	if(*motoB > PWM_MAX) *motoB=PWM_MAX;
	if(*motoB < PWM_MIN) *motoB=PWM_MIN;
}


/*����ֵ����*/
int16 abs(int16 p)
{
	int16 q;
	q=p>0?p:(-p);
	return q;
}

/*��ֵ����*/
/*��ڲ�����PID������ɺ������PWMֵ*/
void Load(int moto1,int moto2)//moto1=-200����ת200������
{	
		if (moto1<0)		
	{
	 pwm_duty(PWM6_P01,abs(moto1));
	 pwm_duty(PWM5_P00,0);
	}
	else
	{
		pwm_duty(PWM5_P00,abs(moto1));
	  pwm_duty(PWM6_P01,0);
	} 
	 
	if (moto2<0)
	{
	 pwm_duty(PWM4P_P26,abs(moto2));
	 pwm_duty(PWM2P_P22,0);
	}	 		

  else
	{
	 pwm_duty(PWM2P_P22,abs(moto2));
	 pwm_duty(PWM4P_P26,0);
	}	
}
