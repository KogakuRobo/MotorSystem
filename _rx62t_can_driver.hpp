#ifndef _rx62t_can_driver_HPP_
#define _rx62t_can_driver_HPP_
#include"CAN.h"
#include"_rx_can_bus.hpp"

class _rx62t_CAN_bus:public _rx_CAN_bus{
	
	static _rx_CAN_bus *This;
	
	static	CAN_MSG msg_buff[32];

	CAN_MSG &GetMsgBuff(int num);
	
	void p_MB_registe_write(int num,CAN_MSG&);
	void p_MB_registe_read(int num,CAN_MSG&);
	void p_MIER_registe_write(int num,int);
	void p_MKIVLR_registe_write(int num,int);
	void p_MCTL_registe_write(int num,unsigned char);
	void p_MCTL_TRMREQ_registe_write(int num,int);
	void p_MCTL_RECREQ_registe_write(int num,int);
	bool p_MCTL_is_use(int num);
	bool p_STR_SDST(void);
	bool p_STR_NDST(void);
	bool p_STR_SENDDATA(int num);
	bool p_STR_NEWDATA(int num);
	
public:
	_rx62t_CAN_bus(void);
	void SetMode(CAN_Mode);
	void SetMask(int num,unsigned short SID,unsigned short EID);
	
static	void CAN0_TXM0(void);
static	void CAN0_RXM0(void);
};

#endif