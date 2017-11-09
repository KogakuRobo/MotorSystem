#include "ManualControl.hpp"
#include <string.h>

ManualControl::ManualControl(MotorSystem *g_hw):
mode(INITIALIZE_MODE),
hw(g_hw)
{
}

void ManualControl::Run(void)
{
	switch(InputOperation()){
	case TEST:
		MotorSystemBegin();
		break;
	case SET_VELOCITY:
		set_velocity();
		break;
	}
}

void ManualControl::MotorSystemBegin(void){
	hw->Begin();
	
	hw->rpc = 3.141592 / 2.0 / 500;
	hw->Vcc = 12;
	hw->Kt = RZ735VA_9517_Kt;
	//hw->Kt = MAXON_RE40_24V_Kt;
	hw->SetDuty(0);
	//6while(!hw->Calibration());
	//‘«‰ñ‚èŽÀŒ±‹@—pPIDƒQƒCƒ“
	/*
	hw->Current_PID.SetPGain(4.2);
	hw->Current_PID.SetIGain(2.0);
	hw->Current_PID.SetDGain(0);
	hw->Velocity_PID.SetPGain(2.0);
	hw->Velocity_PID.SetIGain(2.0);
	hw->Velocity_PID.SetDGain(0);
	*/
	
	//735
	/*/
	hw->Current_PID.SetPGain(2.0);
	hw->Current_PID.SetIGain(0.8);
	hw->Current_PID.SetDGain(50.0*0.000002);
	hw->Velocity_PID.SetPGain(1.5);
	hw->Velocity_PID.SetIGain(1.0);
	hw->Velocity_PID.SetDGain(0.0*0.000002);
	//*/
	
	//540
	//*/
	hw->Current_PID.SetPGain(7.5);
	hw->Current_PID.SetIGain(1.5);
	hw->Current_PID.SetDGain(0.1*0.000002);
	hw->Velocity_PID.SetPGain(1.5);
	hw->Velocity_PID.SetIGain(0.4);
	hw->Velocity_PID.SetDGain(0.0*0.000002);
	//*/
	
	//hw->Velocity_PID.SetPGain(0.10);
	//hw->Velocity_PID.SetIGain(0.01);
	//hw->Velocity_PID.SetDGain(0.00002);
}

void  ManualControl::set_velocity(void){
	float v;
	printf("Set Velocity:");
	scanf("%f",&v);
	
}

struct{
	int code;
	char *description;
}operation_cmd[] = {
	{ ManualControl::TEST,"In test mode. Hardware initializing.\n\r"},
	{ ManualControl::SET_VELOCITY,"Input Velocity Refarence.\n\r"}
};

int ManualControl::InputOperation(void){
	int in;
	for(int i = 0;i < MAX_CODE;i++){
		printf("%d:%s",i,operation_cmd[i].description);
	}
	printf("Set command:");
	scanf("%d",&in);
	fflush(stdin);
	return in;
}