#include"MotorSystem.h"
#include"iodefine.h"
#include"CAN.h"

void OSC_Init(void);
void MSTP_Init(void);
void WDT_Init(void);
void GPIO_Init(void);
void GPT_Init(void);
void MTU_Init(void);
void ADC_Init(void);

MotorSystem::MotorSystem(void):
Current_PID	(2.6,		0.8,		0.0,		0.00002),
Velocity_PID	(2.2,		0.02,		0.0,		0.0002 )
{
	this->mode = STOP;	//MotorSystemのモード
	this->e_mode = NON_ERROR;	//エラー識別子
	this->is_mode = END;		//イニシャライズのサブモード
		
//	定数設定
	this->rpc = 3.1415 / 2.0 / 500 ;	//エンコーダ初期設定
	this->Kt = MAXON_RE40_24V_Kt;		//トルク定数設定
	this->Vcc = 24;				//電源電圧設定
	
	T_ref = 0;				
	V_ref = 0;
	Vo_ref = 0;
	C_ref = 0;
	current_offset = 0;
	velocity = 0;
	
//	ハードウェア初期化
	OSC_Init();			//クロック設定
	MSTP_Init();			//消費電流提言機能設定
	WDT_Init();			//ウォッチドックタイマ設定
	GPIO_Init();			//汎用入出力機能設定
	GPT_Init();			//汎用PWMタイマ設定
	MTU_Init();			//MTU機能設定
	ADC_Init();			//ADC設定

//	初期化処理
	SetDuty(0);				//出力　0
	
//CAN通信初期化
	can_bus.SetMode(CANM_OPERATION);
	can_bus.SetMask(0,0x000f,0x0000);	//コマンド受信用マスク
	can_bus.SetMask(1,0x0000,0x0000);
	can_bus.SetMask(2,0x0000,0x0000);
	can_bus.SetMask(3,0x0000,0x0000);
	can_bus.SetMask(4,0x0000,0x0000);
	can_bus.SetMask(5,0x0000,0x0000);
	can_bus.SetMask(6,0x0000,0x0000);
	can_bus.SetMask(7,0x0000,0x0000);
	
//CAN 受信設定
	CAN_MSG msg;
	msg.SID = (~PORT9.PORT.BYTE >> 1) & 0x0f;
	msg.IDE = 0;
	msg.RTR = 0;
	msg.attr = (void *)this;
	msg.handle = MotorSystem::NormalCommandHandle;
	
	can_bus.ReceiveSet(msg,0,0);
	msg.RTR = 1;
	can_bus.ReceiveSet(msg,1,0);
	
//MTUクロックスタート（ただし、割り込みは生成しない。）
	MTU_ClockStart();
}

void OSC_Init(void)
{
	//EXTAL = 12.5MHz
	
	//システムクロック設定
	//クロック設定を確実にするために確認書き込みをおこなう。
	while(!(SYSTEM.SCKCR.BIT.ICK == 0))SYSTEM.SCKCR.BIT.ICK = 0;	//ICK = EXTAL * 8 = 100MHz
	while(!(SYSTEM.SCKCR.BIT.PCK == 1))SYSTEM.SCKCR.BIT.PCK = 1;	//PCK = EXTAL * 4 =  50MHz
}

void WDT_Init(void)
{
	WDT.WRITE.WINB = 0x5A5F;	//WDT設定 WDT.RSTCSR = 5F,
	/*
	
	
	*/
}

void MSTP_Init(void)
{
	MSTP(GPT0) = 0;
	MSTP(MTU0) = 0;
	MSTP(MTU1) = 0;
	MSTP(MTU2) = 0;
	MSTP(S12AD0) = 0;
}

void GPIO_Init(void)
{
	PORT2.DDR.BIT.B2 = 1;
	PORT2.DDR.BIT.B3 = 1;
	PORT2.DDR.BIT.B4 = 1;
	PORT9.DDR.BYTE = 0x00;
}

void GPT_Init(void)
{
	GPT.GTWP.BIT.WP0 = 0;
	
	GPT0.GTIOR.BIT.OADFLT = 0;
	GPT0.GTIOR.BIT.OBDFLT = 0;
	GPT0.GTIOR.BIT.OAHLD = 0;
	GPT0.GTIOR.BIT.OBHLD = 0;
	GPT0.GTIOR.BIT.GTIOA = 0x09;
	GPT0.GTIOR.BIT.GTIOB = 0x09;
	
	GPT0.GTCR.BIT.MD = 0;
	GPT0.GTCR.BIT.TPCS = 0;
	GPT0.GTCR.BIT.CCLR = 0;
	
	PORT7.DDR.BIT.B1 = 1;
	PORT7.DDR.BIT.B4 = 1;
	
	GPT0.GTPR = 1000;	//100kHz
	//GPT0.GTPR = 2500;	//40kHz
	GPT0.GTCCRA = 0;
	GPT0.GTCCRB = 0;	
	
}

//エンコーダ入力・処理初期化
void MTU0_1_Init(void)
{
	MTU0.TCR.BYTE = 0xa8;		//カウントアップ速度100MHz(100MHz)
	//MTU0.TCR.BYTE = 0xa9;		//カウントアップ速度100MHz(100MHz)
	MTU0.TMDR1.BYTE = 0x20;		//BDバッファモード
	MTU0.TCNT = 0;			//クリア
	MTU0.TGRA = 10000 - 1;
	MTU0.TGRC = 20000 - 1;		//0.2ms周期
	//MTU0.TGRA = 12500 - 1;
	//MTU0.TGRC = 25000 - 1;	//1ms周期
	MTU0.TGRB = 0;
	MTU0.TGRD = 0;
	
	MTU0.TIORH.BIT.IOB = 0xf;
	
	IPR(MTU0,TGIA0) = 15;
	IEN(MTU0,TGIA0) = 1;
	
	IPR(MTU0,TGIC0) = 15;
	IEN(MTU0,TGIC0) = 1;
	
	MTU0.TIER.BIT.TGIEA = 1;
	MTU0.TIER.BIT.TGIEC = 1;
	
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

//AD変換開始トリガ＆電流制御生成
void MTU2_Init(void)
{
	MTU2.TCR.BYTE = 0x20;
	MTU2.TMDR1.BIT.MD = 0;
	
	//MTU2.TGRA = 10000 - 1;	//735
	MTU2.TGRA = 2000 - 1;		//Maxon		
	
	MTU2.TIOR.BYTE = 0x00;
	
	IPR(MTU2,TGIA2) = 14;
	IEN(MTU2,TGIA2) = 1;
	
	MTU2.TIER.BIT.TGIEA = 1;
	MTU2.TIER.BIT.TTGE = 1;
}

void MTU_Init(void)
{
	MTU0_1_Init();
	MTU2_Init();
}

void ADC_Init(void)
{
	S12AD0.ADCSR.BIT.EXTRG = 0;
	S12AD0.ADCSR.BIT.TRGE = 1;
	S12AD0.ADCSR.BIT.CKS = 3;
	S12AD0.ADCSR.BIT.ADIE = 1;
	S12AD0.ADCSR.BIT.ADCS = 0;
	
	S12AD0.ADSSTR = 0x20;
	
	IPR(S12AD0,S12ADI0) = 13;
	IEN(S12AD0,S12ADI0) = 1;
	
	S12AD0.ADSTRGR.BIT.ADSTRS0 = 0x03;
	
	S12AD0.ADANS.BIT.CH = 0;
}
