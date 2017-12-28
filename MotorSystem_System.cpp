#include"MotorSystem.h"

MotorSystem_Mode MotorSystem::GetMode(void)
{
	return state.mode;
}

#define IS_EQUAL(x,y) (x == y)
#define IS_ACTION(x) (IS_EQUAL(x,DUTY) || IS_EQUAL(x,TORQUE) || IS_EQUAL(x,VELOCITY) || IS_EQUAL(x,POSITION))
#define IS_PAUSE(x) (IS_EQUAL(x,INITIALIZE) || IS_EQUAL(x,STOP))
#define IS_ERROR(x) (IS_EQUAL(x,ERROR))

void MotorSystem::SetMode(MotorSystem_Mode m)
{
	
	if(this->IsError())return;	//エラーモードであれば、モード変更を許可しない
	
	
	if(IS_ERROR(m)){
		wdt.stop();
		GPT_ClockStop();
		
		MTU_ClockStop();
		//AllControlStop();
		Current_PID.SumReset();
		Velocity_PID.SumReset();
		SetDuty(0);
	}
	else if(IS_PAUSE(m)){
		wdt.stop();
		//AllControlStop();
		Current_PID.SumReset();
		Velocity_PID.SumReset();
		SetDuty(0);
	}
	else if(IS_ACTION(m)){
		wdt.start();
		//AllControlStart();
	}
	
	this->state.mode = m;
	
}

void MotorSystem::SetMode(MotorSystem_ErrorMode m)
{
	this->state.e_mode = m;
	SetMode(ERROR);
	switch(m){
	case OVER_DUTY:
	case OVER_VOLTAGE:
		this->SetDuty(0);
		break;
	default:		//case NON_ERROR
		return ;
	}
}

bool MotorSystem::IsPause(void)
{
	return IS_PAUSE(this->state.mode);
}

bool MotorSystem::IsAction(void)
{
	return IS_ACTION(this->state.mode);
}

bool MotorSystem::IsError(void)
{
	return IS_ERROR(this->state.mode);
}

bool MotorSystem::Calibration(void)
{
	switch(state.is_mode){
	case START:
		CurrentControlStart();
		state.is_mode = CURRENT_OFFSET_CALCULATION;
		break;
		
	case CURRENT_OFFSET_CALCULATION:
		break;
		
	case CURRENT_OFFSET_CALCULATION_END:
		state.is_mode = END;
		break;
		
	default:
		break;
	}
	return IS_EQUAL(state.is_mode,END);
}

void MotorSystem::SetVoltage(float v)
{
	this->Vo_ref = v;			//電圧目標値保存
	if(v > this->Vcc){
		//SetMode(OVER_VOLTAGE);
		//return;
	}
	this->SetDuty(v / this->Vcc * 100);	//電圧目標値を電源電圧で割り、dutyに変える
}

void MotorSystem::SetVelocity(float vel)
{
	//if((vel < 4.0) && (vel > -4.0)){
	//	SetMode(STOP);
	//	AllControlStop();
	//	Current_PID.SumReset();
	//	Velocity_PID.SumReset();
	//	SetDuty(0);
	//	return;
	//}
	//if(vel > this->velocity_limit)vel = this->velocity_limit;
	
	if((this->state.mode == STOP) || (this->state.mode == INITIALIZE)){
		this->V_ref = vel;
		SetMode(VELOCITY);
	}
	else{
		this->V_ref = vel;
		return;
	}
}

void MotorSystem::SetTorque(float t)
{
	
	if((this->state.mode == STOP) || (this->state.mode == INITIALIZE)){
		this->T_ref = t;
		SetMode(TORQUE);
		AllControlStart();
	}
	else{
		this->T_ref = t;
		return;
	}
}