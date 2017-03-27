#include"_rx_can_bus.hpp"
#include<stddef.h>

int _rx_CAN_bus::Send(CAN_MSG msg)
{
	int i;
	for(i =0;i < 32;i++){
		if(!p_MCTL_is_use(i)){
			p_MB_registe_write(i,msg);
			
			if(msg.handle == NULL)
				p_MIER_registe_write(i,0);
			else
				p_MIER_registe_write(i,1);
	
			msg.other_data = i;
			msg.bus = this;
			GetMsgBuff(i) = msg;
			p_MCTL_TRMREQ_registe_write(i,1);
			return 0;	
		}
	}
	return -1;
}

int _rx_CAN_bus::ReceiveSet(CAN_MSG msg,bool mask_e)
{
	for(int i =0;i < 32;i++){
		if(!p_MCTL_is_use(i)){
			p_MB_registe_write(i,msg);
			if(msg.handle == NULL)
				p_MIER_registe_write(i,0);
			else
				p_MIER_registe_write(i,1);
	
			msg.other_data = i;
			msg.bus = this;
			GetMsgBuff(i) = msg;
			
			p_MKIVLR_registe_write(i,mask_e);
			p_MCTL_RECREQ_registe_write(i,1);
			return 0;	
		}
	}
	return -1;
}

int _rx_CAN_bus::ReceiveSet(CAN_MSG msg,unsigned short num,bool mask_e)
{
	if(num > 31)return -1;
	
	if(!p_MCTL_is_use(num)){
		p_MB_registe_write(num,msg);
		if(msg.handle == NULL)
				p_MIER_registe_write(num,0);
		else
				p_MIER_registe_write(num,1);
	
		msg.other_data = num;
		msg.bus = this;
		GetMsgBuff(num) = msg;
		
		p_MKIVLR_registe_write(num, mask_e);
		//p_MCTL_RECREQ_registe_write(num,1);
		p_MCTL_registe_write(num, 0x40);
		return 0;	
	}
	return -1;
}

HandleReturn _rx_CAN_bus::HandleCall(int num)
{
	CAN_MSG msg = GetMsgBuff(num);
	if(msg.handle != NULL){
		
		this->p_MB_registe_read(num,msg);
		return msg.handle(msg);
	}
	return NON_REQUEST;
}

void _rx_CAN_bus::RXM(void)
{
	while(p_STR_NDST() == 1){
		for(int i=0;i < 32;i++){
			if(p_STR_NEWDATA(i) == 1){
				switch(this->HandleCall(i)){
				case RX_RESET:
					p_MCTL_registe_write(i,0x40);
					if(p_STR_NDST() == 1)continue;
					else return;
				}
			}
		}
	}
	return;
}

void _rx_CAN_bus::TXM(void)
{
	while(p_STR_SDST() == 1){
		for(int i=0;i < 32;i++){
			if(p_STR_SENDDATA(i) == 1){
				switch(this->HandleCall(i)){
				case NON_REQUEST:
					p_MCTL_TRMREQ_registe_write(i,0);
					p_MCTL_registe_write(i,0x00);
					if(p_STR_SDST() == 1)continue;
					else return;
				default:
					break;
				}
			}
		}
	}
	return;
}
