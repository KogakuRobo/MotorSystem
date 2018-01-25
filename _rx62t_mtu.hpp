#ifndef _rx62t_mtu_HPP_
#define _rx62t_mtu_HPP_

class _rx62t_MTU0{
	//カウントアップ周波数[Hz]
	long timerFreq;
	static const int TPSC[];
	long clockRate;
public:
	_rx62t_MTU0(void);
	void begin(void);
	void SetFrequency(long Hz);
	
	void EnableTGIA(bool = true);
	void EnableTGIC(bool = true);
	
	long GetClockRate(void);
	long GetInterruptRate(void);
	
	void Start(void);
	void Stop(void);
};

class _rx62t_MTU1{
public:
	_rx62t_MTU1(void);
	void begin(void);
	
	void Start(void);
	void Stop(void);
};

class _rx62t_MTU2{
	long timerFreq;
	static const int TPSC[];
	long clockRate;
public:
	_rx62t_MTU2(void);
	void begin(void);
	void SetFrequency(long Hz);
	
	long GetInterruptRate(void);
	
	void Start(void);
	void Stop(void);
};

#endif