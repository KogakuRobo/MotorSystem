#include"_rx62t_mtu.hpp"
#include"iodefine.h"
#include"MotorSystem_Define.h"

const int _rx62t_MTU0::TPSC[] = {1,4,16,64};

_rx62t_MTU0::_rx62t_MTU0(void)
{
	MSTP(MTU0) = 0;
}

void _rx62t_MTU0::begin(void)
{
	MTU0.TCR.BIT.CCLR = 0x5;	//TGRCでカウントクリア
	MTU0.TCR.BIT.CKEG = 1;		//下りエッジでカウントアップ
	MTU0.TMDR1.BYTE = 0x20;		//BDバッファモード
	MTU0.TCNT = 0;			//クリア
	this->SetFrequency(1000);	//RZ735=1khz,Maxon = 5kHz
	MTU0.TGRB = 0;			//初期クリア
	MTU0.TGRD = 0;			//初期クリア
	
	MTU0.TIORH.BIT.IOB = 0xf;	//MTU1のカウントアップでTCNTインプットキャプチャ
	
	IPR(MTU0,TGIA0) = 15;
	IEN(MTU0,TGIA0) = 1;
	
	IPR(MTU0,TGIC0) = 15;
	IEN(MTU0,TGIC0) = 1;
	
	MTU0.TIER.BIT.TGIEA = 1;
	MTU0.TIER.BIT.TGIEC = 1;
}

void _rx62t_MTU0::SetFrequency(long Hz)
{
	const char TPSC_num = 4;
	
	for(int i = 0;i < TPSC_num;i++){
		float t = (ICK_CLOCK * 1000000.0) / (Hz * TPSC[i]);
		if(t < 65536/* 2^16 */){
			clockRate = (ICK_CLOCK * 1000000.0) / TPSC[i];
			MTU0.TCR.BIT.TPSC = i;
			MTU0.TGRC = (unsigned short)t - 1;
			MTU0.TGRA = MTU0.TGRC / 2;
			timerFreq = Hz;
			return;
		}
	}
}

void _rx62t_MTU0::EnableTGIA(bool b)
{
	MTU0.TIER.BIT.TGIEA = b;
}

void _rx62t_MTU0::EnableTGIC(bool b)
{
	MTU0.TIER.BIT.TGIEC = b;
}

long _rx62t_MTU0::GetClockRate(void)
{
	return clockRate;
}

long _rx62t_MTU0::GetInterruptRate(void)
{
	return timerFreq;
}

void _rx62t_MTU0::Start(void)
{
	MTU.TSTRA.BIT.CST0 = 1;
}

void _rx62t_MTU0::Stop(void)
{
	MTU.TSTRA.BIT.CST0 = 0;
}

_rx62t_MTU1::_rx62t_MTU1(void)
{
	MSTP(MTU1) = 0;
}

void _rx62t_MTU1::begin(void)
{	
	MTU1.TMDR1.BYTE = 0x04;
	MTU1.TCNT = 0;
	
	MTU1.TIOR.BYTE = 0xff;
	
	PORT3.ICR.BIT.B2 = 1;
	PORT3.ICR.BIT.B3 = 1;
	
	IPR(MTU1,TCIU1) = 15;
	IEN(MTU1,TCIU1) = 1;
	
	IPR(MTU1,TCIV1) = 15;
	IEN(MTU1,TCIV1) = 1;
	
	MTU1.TIER.BIT.TCIEV = 1;
	MTU1.TIER.BIT.TCIEU = 1;
}

void _rx62t_MTU1::Start(void)
{
	MTU.TSTRA.BIT.CST1 = 1;
}

void _rx62t_MTU1::Stop(void)
{
	MTU.TSTRA.BIT.CST1 = 0;
}

const int _rx62t_MTU2::TPSC[] = {1,4,16,64,1,1,1,1024};

_rx62t_MTU2::_rx62t_MTU2(void)
{
	MSTP(MTU2) = 0;
}

void _rx62t_MTU2::begin(void)
{	
	MTU2.TCR.BIT.CCLR = 1;
	MTU2.TMDR1.BIT.MD = 0;
	
	//MTU2.TGRA = 10000 - 1;	//735
	//MTU2.TGRA = 2000 - 1;		//Maxon		
	SetFrequency(10000);
	
	MTU2.TIOR.BYTE = 0x00;
	
	IPR(MTU2,TGIA2) = 14;
	IEN(MTU2,TGIA2) = 1;
	
	MTU2.TIER.BIT.TGIEA = 1;
	MTU2.TIER.BIT.TTGE = 1;
}

void _rx62t_MTU2::SetFrequency(long Hz)
{
	const char TPSC_num = 8;
	
	for(int i = 0;i < TPSC_num;i++){
		float t = (ICK_CLOCK * 1000000.0) / (Hz * TPSC[i]);
		if(t < 65536/* 2^16 */){
			clockRate = (ICK_CLOCK * 1000000.0) / TPSC[i];
			MTU2.TCR.BIT.TPSC = i;
			MTU2.TGRA = (unsigned short)t - 1;
			timerFreq = Hz;
			return;
		}
	}
}

long _rx62t_MTU2::GetInterruptRate(void)
{
	return timerFreq;
}

void _rx62t_MTU2::Start(void)
{
	MTU.TSTRA.BIT.CST2 = 1;
}

void _rx62t_MTU2::Stop(void)
{
	MTU.TSTRA.BIT.CST2 = 0;
}