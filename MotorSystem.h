/*
	This file is MotorSystem ver7 main header.	
*/
#ifndef _MotorSystem_H_
#define _MotorSystem_H_

/*******************************************************************************/
//
//�d�����䊄�荞�ݎ���	0.01ms
//���x���䊄�荞��	0.1ms
/*******************************************************************************/

#include"MotorSystem_Define.h"
#include"CAN.h"
#include"_rx62t_can_driver.hpp"
#include"PID.hpp"

#include<stdio.h>

class MotorSystem{
	
private:
	MotorSystem_Mode mode;
	MotorSystem_ErrorMode e_mode;
	MotorSystem_InitializeSubMode is_mode;
	
	_rx62t_CAN_bus can_bus;
	
private:
	float T_ref;	//torque  �ڕW�l	[mNm]
	float V_ref;	//velocity�ڕW�l	[rad/s]
	float Vo_ref;	//Voltage �ڕW�l	[V]
	float C_ref;	//Current �ڕW�l	[A]
	
	float current;	//current���ݒl		[A]
	float velocity;	//velocity���ݒl	[rad/s]
	
	float current_offset;
public:
	float rpc;	//��J�E���g������̊p�x[rad / count]
	float Kt;	//�g���N�萔[mNm / A]
	float Vcc;	//�d���d��[V]
	
/***********************************************************************/
//	�n�[�h�E�F�A�ˑ�����֐��Q
//private �����o
//	�n�[�h�E�F�A����C���^�[�t�F�[�X���[�`�����g�p���郁���o
/***********************************************************************/
private:

	void WDT_Start(void);
	void WDT_Stop(void);
public:	void WDT_Clear(void);

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
		printf("%f,%f,%f\n",Vo_ref,current,velocity);
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