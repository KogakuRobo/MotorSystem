#include"ACS712.hpp"

ACS712::ACS712(_rx62t_ADC *_adc,unsigned int max_a):adc(_adc),maxA(max_a)
{
}

float ACS712::GetCurrent(void)
{
	float v_data = adc->GetVal() * 5.0;
	float i_data = (v_data - 2.5) * (maxA / 2.0);
	return i_data;
}

float ACS712::GetMax(void)
{
	return maxA;
}