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
	
	SET_VGAIN_K	= 0x48,
	GET_V_K		= 0x78,
	SET_VGAIN_TI	= 0x49,
	GET_V_TI	= 0x79,
	SET_VGAIN_TD	= 0x4a,
	GET_V_TD	= 0x7a,
	
	SET_CGAIN_K	= 0x4c,
	GET_C_K		= 0x7c,
	SET_CGAIN_TI	= 0x4d,
	GET_C_TI	= 0x7d,
	SET_CGAIN_TD	= 0x4e,
	GET_C_TD	= 0x7e,
	
	BEGIN		= 0x44,
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


// GET_*命令のリターンメッセージ関数。
// 同じ処理が並んだので関数化
void return_parameter(CAN_bus &bus,CAN_MSG &msg,float data)
{
	DATA_TRANSER trans;
	trans.FLOAT.f = data;
	msg.data[0] = trans.c_data[0];
	msg.data[1] = trans.c_data[1];
	msg.data[2] = trans.c_data[2];
	msg.data[3] = trans.c_data[3];
	msg.DLC = 4;
	bus.Send(msg);
}

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
		return_parameter(This->can_bus,msg,This->velocity);
		break;
	case GET_TORQUE:
		return_parameter(This->can_bus,msg,This->current * This->Kt);
		break;
	case GET_DUTY:
		return_parameter(This->can_bus,msg,This->Vo_ref / This->Vcc * 100);
		break;
	case GET_CURRENT:
		return_parameter(This->can_bus,msg,This->current);
		break;
	case GET_STATE:
		This->state.MD_Power = 1;
		for(int i = 0;i < 8;i++){
			msg.data[i] = This->state.c_data[i];
		}
		msg.DLC = 8;
		This->can_bus.Send(msg);
		break;
	case SET_VGAIN_K:
		This->Velocity_PID.SetK(trans.FLOAT.f);
		break;
	case GET_V_K:
		return_parameter(This->can_bus,msg,This->Velocity_PID.GetK());
		break;
	case SET_VGAIN_TI:
		This->Velocity_PID.SetTi(trans.FLOAT.f);
		break;
	case GET_V_TI:
		return_parameter(This->can_bus,msg,This->Velocity_PID.GetTi());
		break;
	case SET_VGAIN_TD:
		This->Velocity_PID.SetTd(trans.FLOAT.f);
		break;
	case GET_V_TD:
		return_parameter(This->can_bus,msg,This->Velocity_PID.GetTd());
		break;
	case SET_CGAIN_K:
		This->Current_PID.SetK(trans.FLOAT.f);
		break;
	case GET_C_K:
		return_parameter(This->can_bus,msg,This->Current_PID.GetK());
		break;
	case SET_CGAIN_TI:
		This->Current_PID.SetTi(trans.FLOAT.f);
		break;
	case GET_C_TI:
		return_parameter(This->can_bus,msg,This->Current_PID.GetTi());
		break;
	case SET_CGAIN_TD:
		This->Current_PID.SetTd(trans.FLOAT.f);
		break;
	case GET_C_TD:
		return_parameter(This->can_bus,msg,This->Current_PID.GetTd());
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
	case BEGIN:
		This->Begin();
		msg.RTR = 0;
		This->can_bus.Send(msg);
		break;
	case SET_MODE:
		if(msg.data[0] == STOP){
			This->SetMode((MotorSystem_Mode)msg.data[0]);
		}
		break;
	default:
		break;
	}
	This->wdt.clear();
	PORT2.DR.BIT.B4 = 0;
	return RX_RESET;
}