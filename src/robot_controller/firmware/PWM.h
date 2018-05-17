#ifndef _PWM_H_
#define _PWM_H_

#include <Servo.h>
#include "include.h"
//extern uint16 ServoPwmDutySet[];

//extern bool ServoPwmDutyHaveChange;
class ServoController
{
public:
	ServoController();
	void ServoSetPluseAndTime(uint8 id,uint16 p,uint16 time);
	void ServoPwmDutyCompare();
	void InitPWM();
private:
	uint16 ServoPwmDuty[8] = {1500,1500,1500,1500,1500,1500,1500,1500};	//PWM脉冲宽度
	uint16 ServoPwmDutySet[8] = {1500,1500,1500,1500,1500,1500,1500,1500};	//PWM脉冲宽度
	float ServoPwmDutyInc[8];		//为了速度控制，当PWM脉宽发生变化时，每2.5ms或20ms递增的PWM脉宽

	bool ServoPwmDutyHaveChange = false;	//脉宽有变化标志位	

	uint16 ServoTime = 2000;			//舵机从当前角度运动到指定角度的时间，也就是控制速度

	Servo myservo[6];  // create servo object to control a servo


};

#endif


