#include"MotorSystem.h"
#include"iodefine.h"

#include <math.h>

void MotorSystem::SetDuty(float duty)
{
	const float Max = 95.0;
	const float Min = -95.0;
	bool ret = true;
	
	duty = Limit<float>(duty,Max,Min,ret);//���~�b�g����
	
	if(ret == false){
		//SetMode(OVER_DUTY);
		//return ;
	}
	
	if(duty > 0){
		this->GPT_OAE(1);
		this->GPT_OBE(0);
		this->GPT_ClockStart();
	}
	else if(duty < 0){
		this->GPT_OAE(0);
		this->GPT_OBE(1);
		this->GPT_ClockStart();
		duty *= -1;			//duty�̐����ύX
	}
	else{
		
		this->GPT_OAE(0);
		this->GPT_OBE(0);
		this->GPT_ClockStop();
	}
	
	GPT0.GTCCRA = GPT0.GTPR * duty / 100.0;
	GPT0.GTCCRB = GPT0.GTPR * duty / 100.0;
}

void MotorSystem::i_TorqueControl(void)
{
	
	static float Current,Current_ref;
	static float move_pid;
	
	PORT2.DR.BIT.B2 =1;	
	
	if(this->state.mode == INITIALIZE){
		this->CurrentCalibration();
		PORT2.DR.BIT.B2 =1;
		return;				//�������������Ȃ̂ŁA�d�����T���v�����O���ďI��
	}
	
	Current = this->GetCurrent();					//�d���擾
	this->current = Current;
	
	Current_ref = TorqueToCurrent(this->T_ref);				//�ڕW�d���Z�o
	Current_ref = Limit<float>(Current_ref,17,-17);			//�ڕW�d���Ƀ��~�b�g
	
	this->C_ref = Current_ref;
	
	move_pid = Current_PID.Run(this->current,Current_ref);		//PID����
	
	switch(this->state.mode){
	case TORQUE:
		//SetVoltage(move_pid);					//�g���N����̏ꍇ�A�U���N�d�͕ω��͖�������B(2017/11/12 �g���N����Ƒ��x����ł͓�������Ȃ����ȁj
		//break;
	case VELOCITY:
		SetVoltage(move_pid
			+ velocity * this->Kt / 1000);
			//+ (V_ref) * this->Kt / 1000);	//���x����̏ꍇFF���s���B1000��mNm/A��V s/rad�ɕϊ����邽��
		break;
	case DUTY:
		break;
	default:
		SetVoltage(0);
		break;
	}
	PORT2.DR.BIT.B2 =0;
}

void MotorSystem::i_VelocityControl(void)
{
	float vel;
	static float move_pid = 0;
	
	PORT2.DR.BIT.B3 =1;
	vel = VelocityCalculation();
	
	if(this->state.mode == INITIALIZE){
		//PORT2.DR.BIT.B3 =1;
		return;				//�������������Ȃ̂ŁA�d�����T���v�����O���ďI��
	}
	
	move_pid = Velocity_PID.Run(vel,V_ref);
	switch(this->state.mode){
	case VELOCITY:
		if((V_ref == 0.0) )Velocity_PID.SumReset();//�ڕW�l��0�A�������͕������ω�����ꍇ
		
		if(this->velocity > this->friction_velocity_threshold)
			move_pid += this->dynamic_friction;
		else if(this->velocity < -1 *  this->friction_velocity_threshold)
			move_pid -= this->dynamic_friction;
		else if(this->V_ref > 0)
			move_pid += this->static_friction;
		else if(this->V_ref < 0)
			move_pid -= this->static_friction;
			
		T_ref = move_pid;
		//SetVoltage(move_pid);
		break;
	default:
		break;
	}
	
	PORT2.DR.BIT.B3 =0;
}

void MotorSystem::i_PositionControl(void)
{
}