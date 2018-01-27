#ifndef _ACS712_HPP_
#define _ACS712_HPP_

#include"_rx62t_adc.hpp"

class ACS712
{
private:
	_rx62t_ADC *adc;
	unsigned int maxA;
public:
	ACS712(_rx62t_ADC *_adc,unsigned int max_a);
	
	float GetCurrent(void);
};

#endif