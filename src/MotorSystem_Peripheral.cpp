#include"MotorSystem.h"
#include"iodefine.h"
#include"math.h"
#include"MovingFilter.hpp"

void MotorSystem::MotorFree(void)
{
	PORT7.DR.BIT.B2 = 0;
}

void MotorSystem::MotorUnFree(void)
{
	PORT7.DR.BIT.B2 = 1;
}

void MotorSystem::MTU_ClockStart(void)
{
	if(this->state.mode == ERROR)
		return;	
	mtu0.Start();
	mtu1.Start();
	mtu2.Start();
}

void MotorSystem::MTU_ClockStop(void)
{	
	mtu0.Stop();
	mtu1.Stop();
	mtu2.Stop();
}


#define CURRENT_CALIBRATION_NUMBER 100
void MotorSystem::CurrentCalibration(void)
{
	static float i_buff[CURRENT_CALIBRATION_NUMBER];
	static int num = 0;
	static float sum = 0;
	
	static float i_data;		//電流値
	
	static char flag = 0;
	if(flag == 0){		//一発目の割り込みはAD変換されていなので無視
		flag = 1;
		return ;
	}
	
	i_data = i_data = current_sensor.GetCurrent() ;			//ACS714の30A仕様
	i_buff[num++] = i_data;						//バッファ
	sum += i_data;							//平均値算出のための合計
	
	if(num == CURRENT_CALIBRATION_NUMBER){
		float ave = 0;
		float dev = 0;
		
		ave = sum / CURRENT_CALIBRATION_NUMBER;
		
		for(int j =0;j < CURRENT_CALIBRATION_NUMBER;j++)dev += pow(i_buff[j] - ave,2);
		current_dev = sqrt(dev / (CURRENT_CALIBRATION_NUMBER - 1));
		
		this->current_offset = ave;
		
		state.is_mode = CURRENT_OFFSET_CALCULATION_END;
		this->CurrentControlStop();
		
		sum = 0;
		num = 0;
		flag = 0;
	}
}

float MotorSystem::GetCurrent(void)
{
	//static MovingFilter<float> fil(20);
	static IIR_Filter<float> fil(0.08);
	static float i_data;		//電流値
	
	float temp = current_sensor.GetCurrent();
	
	if(this->current_offset + this->current_dev * 1.0 > abs(temp)){
		i_data = 0.0;
	}else{
		i_data = current_sensor.GetCurrent() - this->current_offset;
	}
	
	if(abs(i_data) > current_sensor.GetMax())
		return 0;
	return fil.Put(i_data);
}

float MotorSystem::GetVelocity(void)
{
	float ret;
	ret = this->velocity;
	return ret;
}

void MotorSystem::PositionControlStart(void)
{
	mtu0.EnableTGIA(true);
}	

void MotorSystem::VelocityControlStart(void)
{
	mtu0.EnableTGIC(true);
}

void MotorSystem::CurrentControlStart(void)
{
	MTU2.TIER.BIT.TTGE = 1;
}

void MotorSystem::PositionControlStop(void)
{
	mtu0.EnableTGIA(false);
}

void MotorSystem::VelocityControlStop(void)
{
	mtu0.EnableTGIC(false);
}

void MotorSystem::CurrentControlStop(void)
{
	MTU2.TIER.BIT.TTGE = 0;
}

void MotorSystem::AllControlStart(void)
{
	PositionControlStart();
	VelocityControlStart();
	CurrentControlStart();
}

void MotorSystem::AllControlStop(void)
{
	PositionControlStop();
	VelocityControlStop();
	CurrentControlStop();
}

#define NUMBER_OF_MOVING_AVERAGE	8
float MotorSystem::VelocityCalculation(void)
{
	float _velocity;
	
	static IIR_Filter<float> fil(0.70);
	
	volatile static unsigned short befor_tgra = 0;
	volatile unsigned short 	MTU0_TGRB = MTU0.TGRB,
			MTU0_TGRD = MTU0.TGRD,
			MTU1_TGRA = MTU1.TGRA,
			MTU1_TGRB = MTU1.TGRB;
	volatile unsigned char TCFD = MTU1.TSR.BIT.TCFD;
	volatile short temp;		//
	volatile float speed = 0.0;
	
	temp = abs(befor_tgra - MTU1_TGRA);//進角方向検出
	
	if(temp == 1){
		speed = this->rpc * mtu0.GetClockRate() / ((unsigned short)(MTU0_TGRB - MTU0_TGRD) + (unsigned short)(MTU0.TGRC-1));
		_velocity = ((TCFD == 1)?1:-1) * speed;
	}
	else if(temp == 0){
		_velocity = 0.0;
	}
	//else if((MTU0_TGRB - MTU0_TGRD) > 158){
		//while(MTU0_TGRB <= MTU0_TGRD){
		//	MTU0_TGRB = MTU0.TGRB;
		//	MTU0_TGRD = MTU0.TGRD;
		//}
	//	speed = this->rpc * (100000000 / 4) / (unsigned short)(MTU0_TGRB - MTU0_TGRD);
	//	_velocity = ((TCFD == 1)?1.0:-1.0) * speed;
	//}
	else{
		speed = this->rpc * (MTU1_TGRA - befor_tgra) * mtu0.GetInterruptRate();
		//_velocity = ((TCFD == 1)?1.0:-1.0) * speed;
	}
	
	befor_tgra = MTU1_TGRA;
	
	if(abs(this->velocity - _velocity) > 500)
		_velocity = this->velocity;
	
	this->velocity = fil.Put(_velocity);
	
	return this->velocity;
	/*
	for(int i = 0;i < NUMBER_OF_MOVING_AVERAGE;i++){
		sum_average += speed_buff[i];
	}
	
	this->velocity = (_velocity + sum_average)/(NUMBER_OF_MOVING_AVERAGE + 1);

	for(int k = 0;k < NUMBER_OF_MOVING_AVERAGE - 1;k++){
		speed_buff[k] = speed_buff[k+1];
	}
	speed_buff[NUMBER_OF_MOVING_AVERAGE-1] = _velocity;
	
	return this->velocity;*/
}

//位置制御割り込み（0.1ms周期　速度制御割り込みと2.5ms差）
#pragma interrupt MTU0_TGIA0(vect=VECT(MTU0,TGIA0),enable)
void MTU0_TGIA0(void)
{
	g_hw->i_PositionControl();
	while(!MTU0.TSR.BIT.TGFA);
	MTU0.TSR.BIT.TGFA=0;
}

//速度制御割り込み（0.1ms周期）
#pragma interrupt MTU0_TGIC0(vect=VECT(MTU0,TGIC0),enable)
void MTU0_TGIC0(void)
{
	g_hw->i_VelocityControl();
	while(!MTU0.TSR.BIT.TGFC);
	while(MTU0.TSR.BIT.TGFC)MTU0.TSR.BIT.TGFC=0;
	IR(MTU0,TGIC0) = 0;
}

//アンダーフロー割り込み
#pragma interrupt MTU1_TCIU1(vect=VECT(MTU1,TCIU1))
void MTU1_TCIU1(void)
{
	while(!MTU1.TSR.BIT.TCFU);
	MTU1.TSR.BIT.TCFU=0;
}

//オーバーフロー割り込み
#pragma interrupt MTU1_TCIV1(vect=VECT(MTU1,TCIV1))
void MTU1_TCIV1(void)
{
	while(!MTU1.TSR.BIT.TCFV);
	MTU1.TSR.BIT.TCFV=0;
}

//電流制御割り込み
#pragma interrupt MTU2_TGIA2(vect=VECT(MTU2,TGIA2))
void MTU2_TGIA2(void)
{
	while(!MTU2.TSR.BIT.TGFA);
	MTU2.TSR.BIT.TGFA=0;
}

#pragma interrupt S12AD0_S12ADI0(vect = VECT(S12AD0,S12ADI0))
void S12AD0_S12ADI0(void)
{
	g_hw->i_TorqueControl();
}