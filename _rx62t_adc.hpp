#ifndef _rx62t_adc_HPP_
#define _rx62t_adc_HPP_

class _rx62t_ADC{
private:
public:
	_rx62t_ADC(void);
	void begin(void);
	
	//0.0 ~ 1.0‚Ì‘Š‘Î•\‹L
	float GetVal(void);
};

#endif