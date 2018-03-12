#include"MotorSystem.h"
#include"iodefine.h"
#include"CAN.h"

void OSC_Init(void);
void GPIO_Init(void);


MotorSystem::MotorSystem(void):
Current_PID	(2.6,		0.8,		0.0,		0.00002),
Velocity_PID	(2.2,		0.02,		0.0,		0.0002 ),
wdt(),
gpt(),
adc(),
mtu0(),
mtu1(),
mtu2(),
current_sensor(&adc,30)
{
	debug_printf("%s[%d]:InConstract",__FILE__,__LINE__);
	
	this->state.mode = INITIALIZE;	//MotorSystemのモード
	this->state.e_mode = NON_ERROR;	//エラー識別子
	this->state.is_mode = START;		//イニシャライズのサブモード
	
	T_ref = 0;				
	V_ref = 0;
	Vo_ref = 0;
	C_ref = 0;
	
	current = 0;
	velocity = 0;
	
	this->DefaultParameter();
	
//	ハードウェア初期化
	OSC_Init();			//クロック設定
	GPIO_Init();			//汎用入出力機能設定
	gpt.begin(100);			//汎用PWMタイマ設定
	mtu0.begin();
	mtu1.begin();
	mtu2.begin();
	adc.begin();
	
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
	msg.SID = (~PORT4.PORT.BYTE >> 4) & 0x0f;
	msg.IDE = 0;
	msg.RTR = 0;
	msg.attr = (void *)this;
	msg.handle = MotorSystem::NormalCommandHandle;
	
	can_bus.ReceiveSet(msg,0,0);
	msg.RTR = 1;
	can_bus.ReceiveSet(msg,1,0);
}

void MotorSystem::DefaultParameter(void)
{
	//	定数設定
	this->rpc = 3.1415 / 2.0 / 500 ;	//エンコーダ初期設定
	this->Kt = MAXON_RE40_24V_Kt;		//トルク定数設定
	this->Vcc = 12;				//電源電圧設定
	
	velocity_limit = 350;
	current_limit = 20;
	
	current_offset = 0;
	
	static_friction = this->CurrentToTorque(0);
	dynamic_friction = this->CurrentToTorque(0);
	friction_velocity_threshold = 0.01;
}

unsigned long MotorSystem::Begin(void)
{

//	初期化処理
	//SetDuty(0);				//出力　0
	//if(PORT1.PORT.BIT.B1 == 1){
	//	printf("Power Down\n");
	//	while(1){
	//	}
	//}
	
//MTUクロックスタート（ただし、割り込みは生成しない。）
	//MTU_ClockStart();
	printf("Begin Running\n");
	CurrentSensor_Init();
	printf("Begin finish\n");
	state.mode = STOP;
	
	return 0;
}

int MotorSystem::CurrentSensor_Init(void){
	state.is_mode = CURRENT_OFFSET_CALCULATION;
	mtu2.Start();
	CurrentControlStart();
	while(state.is_mode != CURRENT_OFFSET_CALCULATION_END);
	mtu2.Stop();
	CurrentControlStop();
	state.is_mode = END;
	return 0;
}

void OSC_Init(void)
{
	//EXTAL = 12.5MHz
	
	//システムクロック設定
	//クロック設定を確実にするために確認書き込みをおこなう。
	while(!(SYSTEM.SCKCR.BIT.ICK == 0))SYSTEM.SCKCR.BIT.ICK = 0;	//ICK = EXTAL * 8 = 100MHz
	while(!(SYSTEM.SCKCR.BIT.PCK == 1))SYSTEM.SCKCR.BIT.PCK = 1;	//PCK = EXTAL * 4 =  50MHz
}

void GPIO_Init(void)
{
	PORT2.DDR.BIT.B2 = 1;
	PORT2.DDR.BIT.B3 = 1;
	PORT2.DDR.BIT.B4 = 1;
	PORT9.DDR.BYTE = 0x00;
}
