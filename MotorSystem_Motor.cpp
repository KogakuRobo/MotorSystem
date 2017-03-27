#include"MotorSystem.h"
#include"iodefine.h"

void MotorSystem::SetDuty(float duty)
{
	const float Max = 95.0;
	const float Min = -95.0;
	bool ret = true;
	
	duty = Limit<float>(duty,Max,Min,ret);//リミット処理
	
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
		duty *= -1;			//dutyの正負変更
	}
	else{
		
		this->GPT_OAE(0);
		this->GPT_OBE(0);
		this->GPT_ClockStop();
	}
	
	GPT0.GTCCRA = duty * 10.0;
	GPT0.GTCCRB = duty * 10.0;
}

void MotorSystem::i_TorqueControl(void)
{
	
	static float Current,Current_ref;
	static float move_pid;
	
	PORT2.DR.BIT.B2 =1;	
	
	if(this->mode == INITIALIZE){
		this->CurrentCalibration();
		PORT2.DR.BIT.B2 =0;
		return;				//初期化処理中なので、電流をサンプリングして終了
	}
	
	Current = this->GetCurrent();					//電流取得
	this->current = Current;
	
	Current_ref = this->T_ref / this->Kt;				//目標電流算出
	Current_ref = Limit<float>(Current_ref,17,-17);			//目標電流にリミット
	
	this->C_ref = Current_ref;
	
	move_pid = Current_PID.Run(Current,Current_ref);		//PID制御
	
	switch(this->mode){
	case TORQUE:
		SetVoltage(move_pid);					//トルク制御の場合、誘導起電力変化は無視する。
		break;
	case VELOCITY:
		//SetVoltage(move_pid);
			//+ (V_ref) * MAXON_RE40_24V_Kt / 1000);	//速度制御の場合FFを行う。1000はmNm/AをV s/radに変換するため
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
	
	if(this->mode == INITIALIZE){
		PORT2.DR.BIT.B3 =0;
		return;				//初期化処理中なので、電流をサンプリングして終了
	}
	
	move_pid = Velocity_PID.Run(vel,V_ref);
	move_pid += this->V_ref * this->Kt / 1000.0;	
	switch(this->mode){
	case VELOCITY:
		SetVoltage(move_pid);
		//T_ref = move_pid;
		break;
	default:
		break;
	}
	
	PORT2.DR.BIT.B3 =0;
}

void MotorSystem::i_PositionControl(void)
{
}