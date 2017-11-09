#include"MotorSystem.h"
#include<stddef.h>
#include"iodefine.h"

//コマンドコード　7bit
//[6] - [5] 優先度(00b:通常、01b:予約、10b:デバッグ・調整、11b:監視
//      [4] R/W (R = 1,W = 0)
//[3] - [0] 識別子
typedef enum{
	SET_VELOCITY	= 0x00,
	GET_VELOCITY	= 0x70,
	SET_TORQUE	= 0x01,
	GET_TORQUE	= 0x71,
	SET_DUTY	= 0x02,
	GET_DUTY	= 0x72,
	GET_CURRENT	= 0x73,
	
	SET_MODE	= 0x04,
	GET_MODE	= 0x14,
	GET_STATE	= 0x74,
	
	SET_VCC		= 0x45,
	SET_PPR		= 0x46,
	SET_KT		= 0x47,
	
	SET_VGAIN_P	= 0x48,
	GET_V_P		= 0x78,
	SET_VGAIN_I	= 0x49,
	GET_V_I		= 0x79,
	SET_VGAIN_D	= 0x4a,
	GET_V_D		= 0x7a,
	
	SET_CGAIN_P	= 0x4c,
	GET_C_P		= 0x7c,
	SET_CGAIN_I	= 0x4d,
	GET_C_I		= 0x7d,
	SET_CGAIN_D	= 0x4e,
	GET_C_D		= 0x7e,
	
}MotorSystem_CMD;//IDの上位7bit分

#pragma pack
typedef union{
	char c_data[8];
	struct{
		float f;
		char nc[4];
	}FLOAT;
}DATA_TRANSER;
#pragma packoption

HandleReturn MotorSystem::SendHandle(CAN_MSG msg)
{
	return NON_REQUEST;
}

HandleReturn MotorSystem::NormalCommandHandle(CAN_MSG msg)
{
	MotorSystem *This = (MotorSystem *)msg.attr;
	PORT2.DR.BIT.B4 = 1;
	
	DATA_TRANSER trans;
	
	for(int i =0;i < msg.DLC;i++){
		trans.c_data[i] = msg.data[i];
	}
	msg.handle = SendHandle;
	msg.RTR = 0;
	
	switch(msg.SID >> 4){
	case SET_VELOCITY:
		This->SetVelocity(trans.FLOAT.f);
		break;
	case SET_TORQUE:
		This->SetTorque(trans.FLOAT.f);
		break;
	case SET_DUTY:
		This->SetDuty(trans.FLOAT.f);
		break;
	case GET_VELOCITY:
		trans.FLOAT.f = This->velocity;
		msg.data[0] = trans.c_data[0];
		msg.data[1] = trans.c_data[1];
		msg.data[2] = trans.c_data[2];
		msg.data[3] = trans.c_data[3];
		msg.DLC = 4;
		This->can_bus.Send(msg);
		break;
	case GET_TORQUE:
		trans.FLOAT.f = This->current * This->Kt;
		msg.data[0] = trans.c_data[0];
		msg.data[1] = trans.c_data[1];
		msg.data[2] = trans.c_data[2];
		msg.data[3] = trans.c_data[3];
		msg.DLC = 4;
		This->can_bus.Send(msg);
		break;
	case GET_DUTY:
		trans.FLOAT.f = This->Vo_ref / This->Vcc * 100;
		msg.data[0] = trans.c_data[0];
		msg.data[1] = trans.c_data[1];
		msg.data[2] = trans.c_data[2];
		msg.data[3] = trans.c_data[3];
		msg.DLC = 4;
		This->can_bus.Send(msg);
		break;
	case GET_CURRENT:
		trans.FLOAT.f = This->current;
		msg.data[0] = trans.c_data[0];
		msg.data[1] = trans.c_data[1];
		msg.data[2] = trans.c_data[2];
		msg.data[3] = trans.c_data[3];
		msg.DLC = 4;
		This->can_bus.Send(msg);
		break;
	case SET_VGAIN_P:
		This->Velocity_PID.SetPGain(trans.FLOAT.f);
		break;
	case SET_VGAIN_I:
		This->Velocity_PID.SetIGain(trans.FLOAT.f);
		break;
	case SET_VGAIN_D:
		This->Velocity_PID.SetDGain(trans.FLOAT.f);
		break;
	case SET_CGAIN_P:
		This->Current_PID.SetPGain(trans.FLOAT.f);
		break;
	case SET_CGAIN_I:
		This->Current_PID.SetIGain(trans.FLOAT.f);
		break;
	case SET_CGAIN_D:
		This->Current_PID.SetDGain(trans.FLOAT.f);
		break;
	case SET_VCC:
		This->Vcc = trans.FLOAT.f;
		break;
	case SET_PPR:
		This->rpc = 3.1415 / 2.0 / trans.FLOAT.f;
		break;
	case SET_KT:
		This->Kt = trans.FLOAT.f;
		break;
	default:
		break;
	}
	This->WDT_Clear();
	PORT2.DR.BIT.B4 = 0;
	return RX_RESET;
}