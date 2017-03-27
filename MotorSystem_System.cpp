#include"MotorSystem.h"

MotorSystem_Mode MotorSystem::GetMode(void)
{
	return mode;
}

#define IS_EQUAL(x,y) (x == y)
#define IS_ACTION(x) (IS_EQUAL(x,DUTY) || IS_EQUAL(x,TORQUE) || IS_EQUAL(x,VELOCITY) || IS_EQUAL(x,POSITION))
#define IS_PAUSE(x) (IS_EQUAL(x,INITIALIZE) || IS_EQUAL(x,STOP))
#define IS_ERROR(x) (IS_EQUAL(x,ERROR))

void MotorSystem::SetMode(MotorSystem_Mode m)
{
	
	if(this->IsError())return;	//エラーモードであれば、モード変更を許可しない
	
	
	if(IS_ERROR(m)){
		this->WDT_Stop();
		GPT_ClockStop();
		
		MTU_ClockStop();
		AllControlStop();
		Current_PID.SumReset();
		Velocity_PID.SumReset();
		SetDuty(0);
	}
	else if(IS_PAUSE(m)){
		this->WDT_Stop();
		AllControlStop();
		Current_PID.SumReset();
		Velocity_PID.SumReset();
		SetDuty(0);
	}
	else if(IS_ACTION(m)){
		this->WDT_Start();
		AllControlStart();
	}
	
	this->mode = m;
	
}

void MotorSystem::SetMode(MotorSystem_ErrorMode m)
{
	this->e_mode = m;
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
	return IS_PAUSE(this->mode);
}

bool MotorSystem::IsAction(void)
{
	return IS_ACTION(this->mode);
}

bool MotorSystem::IsError(void)
{
	return IS_ERROR(this->mode);
}

bool MotorSystem::Calibration(void)
{
	switch(is_mode){
	case START:
		CurrentControlStart();
		is_mode = CURRENT_OFFSET_CALCULATION;
		break;
		
	case CURRENT_OFFSET_CALCULATION:
		break;
		
	case CURRENT_OFFSET_CALCULATION_END:
		is_mode = END;
		break;
		
	default:
		break;
	}
	return IS_EQUAL(is_mode,END);
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
	if((vel < 4.0) && (vel > -4.0)){
		SetMode(STOP);
		AllControlStop();
		Current_PID.SumReset();
		Velocity_PID.SumReset();
		SetDuty(0);
		return;
	}
	else if((this->mode == STOP) || (this->mode == INITIALIZE)){
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
	
	if((this->mode == STOP) || (this->mode == INITIALIZE)){
		this->T_ref = t;
		SetMode(TORQUE);
		AllControlStart();
	}
	else{
		this->T_ref = t;
		return;
	}
}