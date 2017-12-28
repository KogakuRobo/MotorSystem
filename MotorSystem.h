/*
	This file is MotorSystem ver7 main header.	
*/
#ifndef _MotorSystem_H_
#define _MotorSystem_H_

/*******************************************************************************/
//
//電流制御割り込み周期	0.01ms
//速度制御割り込み	0.1ms
/*******************************************************************************/

#include"MotorSystem_Define.h"
#include"CAN.h"
#include"_rx62t_can_driver.hpp"
#include"_rx62t_wdt.hpp"
#include"PID.hpp"

#include<stdio.h>

class MotorSystem{
	
private:
	/*/
	volatile MotorSystem_Mode mode;
	volatile MotorSystem_ErrorMode e_mode;
	volatile MotorSystem_InitializeSubMode is_mode;
	/*/
	volatile union{
		struct{
			union{
				unsigned char uc_mode;
				MotorSystem_Mode mode;
			};
			union{
				unsigned char uc_e_mode;
				MotorSystem_ErrorMode e_mode;
			};
			union{
				unsigned char uc_is_mode;
				MotorSystem_InitializeSubMode is_mode;
			};
			union{
				unsigned char uc_state;
				struct {
					unsigned char MD_Power:1;
					unsigned char dummy:7;
				};
			};
		};
		unsigned char c_data[8];
	}state;
	//*/
	_rx62t_CAN_bus can_bus;
	
private:
	float T_ref;	//torque  目標値	[mNm]
	float V_ref;	//velocity目標値	[rad/s]
	float Vo_ref;	//Voltage 目標値	[V]
	float C_ref;	//Current 目標値	[A]
	
	float current;	//current現在値		[A]
	float velocity;	//velocity現在値	[rad/s]
	
	float current_offset;	//電流センサのオフセット誤差

public:
	float static_friction;			//静止摩擦力	[mNm]
	float dynamic_friction;			//動摩擦力	[mNm]
	float friction_velocity_threshold;	//摩擦力変化の速度閾値[rad/s]
	
	float velocity_limit;
	float current_limit;
	
	
public:
	float rpc;	//一カウント当たりの角度[rad / count]
	float Kt;	//トルク定数[mNm / A]
	float Vcc;	//電源電圧[V]
	
/***********************************************************************/
//	ハードウェア依存制御関数群
//private メンバ
//	ハードウェア制御インターフェースルーチンが使用するメンバ
/***********************************************************************/
private:
	_rx62t_WDT wdt;
public:
	void WDT_Clear(void){wdt.clear();}

//public:
private:
	void GPT_ClockStart(void);	//MotorSystem_Peripheral.h
	void GPT_ClockStop(void);	//MotorSystem_Peripheral.h
	
	void GPT_OAE(unsigned char);	//MotorSystem_Peripheral.h
	void GPT_OBE(unsigned char);	//MotorSystem_Peripheral.h
	
	float GetCurrent(void);	
	void CurrentCalibration(void);
	float GetVelocity(void);
	
	float VelocityCalculation(void);
	
	void MTU_ClockStart(void);
	void MTU_ClockStop(void);

	void PositionControlStart(void);
	void VelocityControlStart(void);
	void CurrentControlStart(void);
	void PositionControlStop(void);
	void VelocityControlStop(void);
	void CurrentControlStop(void);
	
	void AllControlStart(void);
	void AllControlStop(void);

public:
	MotorSystem(void);	//MotorSystem_Init.cpp
	
	void Begin(void);
	int CurrentSensor_Init(void);
	
	MotorSystem_Mode GetMode(void);
	void SetMode(MotorSystem_Mode);
	void SetMode(MotorSystem_ErrorMode);
	
	bool Calibration(void);
	
	bool IsPause(void);
	bool IsAction(void);
	bool IsError(void);
	
	void SetDuty(float);
	void SetVoltage(float);
	void SetTorque(float);
	void SetVelocity(float);
	
	void Logoutput(void)
	{
		printf("%f,%f,%f,%f,%f,%f\n",Vo_ref,current,velocity,Velocity_PID.Proportion,Velocity_PID.Integration,Velocity_PID.Differentiation);
	}
	
	float TorqueToCurrent(float t)
	{
		return t / this->Kt;
	}
	
	float CurrentToTorque(float i)
	{
		return i * this->Kt;
	}
//トルクコントロールルーチン　電流フィードバック制御を行います。
// T_ref -> (C_ref) -> V_ref
//速度制御モードでは速度のフィードフォアード制御を行います（誘導起電力分の除去）。
public:		void i_TorqueControl(void);
public:		PID<float> Current_PID;
	
public:		void i_VelocityControl(void);
public:		PID<float> Velocity_PID;
	
public:		void i_PositionControl(void);

public:	//通信まわり
	static HandleReturn SendHandle(CAN_MSG);
	static HandleReturn NormalCommandHandle(CAN_MSG);
};

extern MotorSystem *g_hw;

#endif