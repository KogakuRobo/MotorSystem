/*
	This file is MotorSystem ver7 main header.	
*/
#ifndef _MotorSystem_H_
#define _MotorSystem_H_

#include"MotorSystem_Define.h"
#include"CAN.h"
#include"_rx62t_can_driver.hpp"
#include"_rx62t_wdt.hpp"
#include"_rx62t_gpt.hpp"
#include"_rx62t_adc.hpp"
#include"_rx62t_mtu.hpp"
#include"ACS712.hpp"
#include"PID.hpp"

#include<stdio.h>

#ifdef DEBUG
#define debug_printf(x) printf(x)
#elif defined(RELESE)
#define debug_printf(x)
#endif

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
	float T_ref;	//torque  �ڕW�l	[mNm]
	float V_ref;	//velocity�ڕW�l	[rad/s]
	float Vo_ref;	//Voltage �ڕW�l	[V]
	float C_ref;	//Current �ڕW�l	[A]
	
	float current;	//current���ݒl		[A]
	float velocity;	//velocity���ݒl	[rad/s]
	
	float current_offset;	//�d���Z���T�̃I�t�Z�b�g�덷
	float current_dev;	//�d���Z���T�̕s�ΕW���΍�

public:
	float static_friction;			//�Î~���C��	[mNm]
	float dynamic_friction;			//�����C��	[mNm]
	float friction_velocity_threshold;	//���C�͕ω��̑��x臒l[rad/s]
	
	float velocity_limit;
	float current_limit;
	
	
public:
	float rpc;	//��J�E���g������̊p�x[rad / count]
	float Kt;	//�g���N�萔[mNm / A]
	float Vcc;	//�d���d��[V]
	
/***********************************************************************/
//	�n�[�h�E�F�A�ˑ�����֐��Q
//private �����o
//	�n�[�h�E�F�A����C���^�[�t�F�[�X���[�`�����g�p���郁���o
/***********************************************************************/
public:
	_rx62t_WDT wdt;
	_rx62t_GPT gpt;
	_rx62t_ADC adc;
	_rx62t_MTU0 mtu0;
	_rx62t_MTU1 mtu1;
	_rx62t_MTU2 mtu2;
	ACS712 current_sensor;
public:
	void WDT_Clear(void){wdt.clear();}

//public:
private:

	
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

	
/***********************************************************************/
//	���[�^����֐��Q
//public �����o
//	�ڕW�l�ݒ�֐��Ȃ�
/***********************************************************************/

public:
	MotorSystem(void);	//MotorSystem_Init.cpp
	void DefaultParameter(void);
	
	unsigned long Begin(void);
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
		printf("%f,%f,%f,%f,%f\n",Vo_ref,current,C_ref,velocity,V_ref);
	}
	
	float TorqueToCurrent(float t)
	{
		return t / this->Kt;
	}
	
	float CurrentToTorque(float i)
	{
		return i * this->Kt;
	}
//�g���N�R���g���[�����[�`���@�d���t�B�[�h�o�b�N������s���܂��B
// T_ref -> (C_ref) -> V_ref
//���x���䃂�[�h�ł͑��x�̃t�B�[�h�t�H�A�[�h������s���܂��i�U���N�d�͕��̏����j�B
public:		void i_TorqueControl(void);
public:		PID<float> Current_PID;
	
public:		void i_VelocityControl(void);
public:		PID<float> Velocity_PID;
	
public:		void i_PositionControl(void);

public:	//�ʐM�܂��
	static HandleReturn SendHandle(CAN_MSG);
	static HandleReturn NormalCommandHandle(CAN_MSG);
};

extern MotorSystem *g_hw;

#endif