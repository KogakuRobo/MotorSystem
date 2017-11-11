#ifndef _ManualControl_HPP_
#define _ManualControl_HPP_

#include <stdio.h>
#include "MotorSystem.h"

typedef enum{
	INITIALIZE_MODE,
	WAIT_MODE,
}SEQUENSE_MODE;

class ManualControl{
	MotorSystem *hw;
	
private:
	typedef enum{
		INITIALIZE_MODE,
		WAIT_MODE,
	}SEQUENSE_MODE;
	SEQUENSE_MODE mode;
public:

	typedef enum{
		TEST,
		SET_VELOCITY,
		MAX_CODE,
	}OPERATION_CODE;
	
private:
	int InputOperation(void);

	void MotorSystemBegin(void);
	void set_velocity(void);
	
public:
	ManualControl(MotorSystem *);
	void Run(void);
};

#endif