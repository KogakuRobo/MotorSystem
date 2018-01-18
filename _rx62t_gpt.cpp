#include"_rx62t_gpt.hpp"
#include"iodefine.h"

_rx62t_GPT::_rx62t_GPT(void)
{
	MSTP(GPT0) = 0;
}

void _rx62t_GPT::begin(long frequency_khz)
{
	GPT.GTWP.BIT.WP0 = 0;
	
	GPT0.GTIOR.BIT.OADFLT = 0;
	GPT0.GTIOR.BIT.OBDFLT = 0;
	GPT0.GTIOR.BIT.OAHLD = 0;
	GPT0.GTIOR.BIT.OBHLD = 0;
	GPT0.GTIOR.BIT.GTIOA = 0x09;
	GPT0.GTIOR.BIT.GTIOB = 0x09;
	
	GPT0.GTCR.BIT.MD = 0;
	GPT0.GTCR.BIT.CCLR = 0;
	
	PORT7.DDR.BIT.B1 = 1;
	PORT7.DDR.BIT.B4 = 1;
	
	//GPT0.GTPR = 1000;	//100kHz
	//GPT0.GTPR = 2500;	//40kHz
	SetFrequency(frequency_khz);
	GPT0.GTCCRA = 0;
	GPT0.GTCCRB = 0;	
}

void _rx62t_GPT::ClockStart(void)
{
	GPT.GTSTR.BIT.CST0 = 1;
}

void _rx62t_GPT::ClockStop(void)
{
	GPT.GTSTR.BIT.CST0 = 0;
}

void _rx62t_GPT::SetDuty(float duty)
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
		GPT0.GTONCR.BIT.OAE = 1;
		GPT0.GTONCR.BIT.OBE = 0;
		ClockStart();
	}
	else if(duty < 0){
		GPT0.GTONCR.BIT.OAE = 0;
		GPT0.GTONCR.BIT.OBE = 1;
		ClockStart();
		duty *= -1;			//dutyの正負変更
	}
	else{
		GPT0.GTONCR.BIT.OAE = 0;
		GPT0.GTONCR.BIT.OBE = 0;
		ClockStop();
	}
	
	GPT0.GTCCRA = GPT0.GTPR * duty / 100.0;
	GPT0.GTCCRB = GPT0.GTPR * duty / 100.0;
}

void _rx62t_GPT::SetFrequency(long khz)
{
	const char pcs_number = 4;
	const char pcs[pcs_number] = {1,2,4,8};
	for(int i = 0;i < pcs_number;i++){
		float t = (ICK_CLOCK * 1000) / (khz * pcs[i]);
		if(t < 65536/* 2^16 */){
			float duty_temp = (GPT0.GTCCRA * 100.0) / (float)GPT0.GTPR;
			//this->ClockStop();
			GPT0.GTCR.BIT.TPCS = i;
			GPT0.GTPR = (unsigned short)t;
			//this->SetDuty(duty_temp);
			return;
		}
	}
	
}
