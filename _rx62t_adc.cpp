#include"_rx62t_adc.hpp"
#include"iodefine.h"

_rx62t_ADC::_rx62t_ADC(void)
{
	MSTP(S12AD0) = 0;
}

void _rx62t_ADC::begin(void)
{
	S12AD0.ADCSR.BIT.EXTRG = 0;
	S12AD0.ADCSR.BIT.TRGE = 1;
	S12AD0.ADCSR.BIT.CKS = 3;
	S12AD0.ADCSR.BIT.ADIE = 1;
	S12AD0.ADCSR.BIT.ADCS = 0;
	
	S12AD0.ADSSTR = 0xFF;
	
	IPR(S12AD0,S12ADI0) = 14;
	IEN(S12AD0,S12ADI0) = 1;
	
	S12AD0.ADSTRGR.BIT.ADSTRS0 = 0x03;
	
	S12AD0.ADANS.BIT.CH = 0;
}

float _rx62t_ADC::GetVal(void)
{
	return S12AD0.ADDR0A /(4096.0 - 1.0);
}