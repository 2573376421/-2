#include "motor.h"
#include "zf_pwm.h"
#include "STC8Hxx.h"


/*限幅函数*/
void Limit(int16 *motoA,int16 *motoB)
{
	if(*motoA > PWM_MAX) *motoA=PWM_MAX;
	if(*motoA < PWM_MIN) *motoA=PWM_MIN;
	
	if(*motoB > PWM_MAX) *motoB=PWM_MAX;
	if(*motoB < PWM_MIN) *motoB=PWM_MIN;
}


/*绝对值函数*/
int16 abs(int16 p)
{
	int16 q;
	q=p>0?p:(-p);
	return q;
}

/*赋值函数*/
/*入口参数：PID运算完成后的最终PWM值*/
void Load(int moto1,int moto2)//moto1=-200：反转200个脉冲
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
