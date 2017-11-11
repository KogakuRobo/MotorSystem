#ifndef _rx62t_wdt_HPP_
#define _rx62t_wdt_HPP_

class _rx62t_WDT{

private:
/*/
	void write_rstcsr(unsigned charÅ@WOVF,unsigned char RSTE);
	void write_tcsr(unsigned char TMS,unsigned char TME,unsigned char CKS);
	void write_tcnt(unsigned char count);
//*/
public:
	_rx62t_WDT(void);
	
	void start(void);
	void stop(void);
	void clear(void);
	
};

#endif