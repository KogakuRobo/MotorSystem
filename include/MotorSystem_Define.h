#ifndef _MotorSystem_Define_H_
#define _MotorSystem_Define_H_

#define EXTAL_CLOCK	12.5	//[MHz]
#define ICK_CLOCK	100	//[MHz]
#define PCK_CLOCK	50	//[MHz]

typedef enum{
	INITIALIZE=0,
	DUTY,
	TORQUE,
	VELOCITY,
	POSITION,
	ERROR,
	STOP,
}MotorSystem_Mode;

#define IS_EQUAL(x,y) (x == y)
#define IS_ACTION(x) (IS_EQUAL(x,DUTY) || IS_EQUAL(x,TORQUE) || IS_EQUAL(x,VELOCITY) || IS_EQUAL(x,POSITION))
#define IS_PAUSE(x) (IS_EQUAL(x,INITIALIZE) || IS_EQUAL(x,STOP))
#define IS_ERROR(x) (IS_EQUAL(x,ERROR))

typedef enum{
	NON_ERROR,
	OVER_DUTY,
	OVER_VOLTAGE,
}MotorSystem_ErrorMode;

typedef enum{
	START,
	CURRENT_OFFSET_CALCULATION,
	CURRENT_OFFSET_CALCULATION_END,
	END,
}MotorSystem_InitializeSubMode;

#define MAXON_RE40_24V_Kt	30.2		//mNm/A
#define RZ735VA_9517_Kt		8.11		//mNm/A
#define RZ735VA_8519_Kt		5.80		//mNm/A
#define RS380			4.73

#define RPM(radps)	(radps * 60 / (2 * 3.141592))

template <typename v_type>
v_type Limit(v_type value, v_type Max, v_type Min)
{
	if(value > Max){
		value = Max;
	}
	else if(value < Min){
		value = Min;
	}
	return value;
}	

template <typename v_type>
v_type Limit(v_type value, v_type Max, v_type Min, bool &ret)
{
	if(value > Max){
		value = Max;
		ret = false;
	}
	else if(value < Min){
		value = Min;
		ret = false;
	}
	else{
		ret = true;
	}
	return value;
}	

#endif