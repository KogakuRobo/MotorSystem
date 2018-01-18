#ifndef _rx62t_gpt_HPP_
#define _rx62t_gpt_HPP_

#include"iodefine.h"
#include"MotorSystem_Define.h"

class _rx62t_GPT{
	
	
public:
	_rx62t_GPT(void);
	void begin(long frequency_khz);
	void ClockStart(void);
	void ClockStop(void);
	void SetDuty(float duty);
	void SetFrequency(long khz);
};

#endif