#ifndef _CAN_H_
#define _CAN_H_	

#define CAN_BITRATE		500	//[kbps]

class CAN_bus;

typedef enum{
	NON_REQUEST,
	TX_RESENT,
	RX_RESET,
}HandleReturn;

class CAN_MSG{
public:
	unsigned short SID;
	unsigned short EID;
	unsigned char RTR;
	unsigned char IDE;
	unsigned char DLC;
	unsigned char data[8];

	void Set(unsigned short sid,unsigned short eid,unsigned char rtr,unsigned char ide,unsigned char dlc,unsigned char d[]){
		this->SID = sid;this->EID = eid;
		this->RTR = rtr;this->IDE = ide;
		this->DLC = dlc;
		for(int i = 0;i < dlc;i++)this->data[i] = d[i];
	}
	
	HandleReturn (*handle)(CAN_MSG);
	void *attr;
	
	CAN_bus *bus;
	unsigned long other_data;
};

class CAN_bus{
public:
	virtual int Send(CAN_MSG msg) = 0;
	int ReceiveSet(CAN_MSG msg,unsigned short num,bool mask_e);
};

#endif