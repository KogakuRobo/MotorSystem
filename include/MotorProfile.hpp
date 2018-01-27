#ifndef _MotorProfile_HPP_
#define _MotorProfile_HPP_


//template class
class MotorProfile{
public:
	virtual float GetKt(void) = 0;
	virtual long GetVelocityControlFrequency(void) = 0;
	
};

#endif