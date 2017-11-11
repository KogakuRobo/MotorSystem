#include "_rx62t_wdt.hpp"
#include "iodefine.h"

_rx62t_WDT::_rx62t_WDT(void)
{
	WDT.WRITE.WINB = 0x5A5F;	//WDTê›íË WDT.RSTCSR = 5F,
}

void _rx62t_WDT::start(void)
{
	WDT.WRITE.WINA = 0xA5FF;
}

void _rx62t_WDT::stop(void)
{
	WDT.WRITE.WINA = 0xA598;
}

void _rx62t_WDT::clear(void)
{
	WDT.WRITE.WINA = 0x5A00;
}