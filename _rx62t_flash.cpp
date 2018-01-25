#include"_rx62t_flash.hpp"
#include"iodefine.h"
#include"MotorSystem_Define.h"
/*
class _rx62t_FlashControl{
public:
	int read(char *,unsigned long);
	int write(const char *,unsigned long);
	int flush(void);
	int seek(unsigned long);
};
//*/

const int _rx62t_FlashControl::page_size = 0x80;			//一ページあたり128バイト
const int _rx62t_FlashControl::flash_area_size = 0x0800 * 4;	//flash全体で2K * 4バイト

_rx62t_FlashControl::_rx62t_FlashControl(void)
{
	FLASH.DFLRE0.WORD = 0x2D0F;			//読出し許可
	FLASH.DFLWE0.WORD = 0x1E0F;			//書き込み許可
	page_head = (unsigned char *)0x00100000;
	FLASH.PCKAR.WORD = PCK_CLOCK;
	this->entry_flash_pe();
	{	//クロック通知
		page_head[0] = 0xE9;
		page_head[0] = 0x03;
		((unsigned short *)page_head)[0] = 0x0F0F;
		((unsigned short *)page_head)[0] = 0x0F0F;
		((unsigned short *)page_head)[0] = 0x0F0F;
		page_head[0] = 0xD0;
		while(!FLASH.FSTATR0.BIT.FRDY);				//レディチェック
	}
	this->entry_flash_normal();
	get_page(0);
	
	count = 0;					//読出し初期値0
	page = 0;					//読出し初期値0
}

int _rx62t_FlashControl::read(char *_buff,unsigned long len)
{
	long now_count = count % this->page_size;
	long now_page = (count - now_count) / this->page_size;
	
	for(int i = 0; i < len;i++){
		_buff[i] = buff[now_count++];
		if(++count == this->page_size){
			flush();
			page++;
			get_page(page);
			now_count = 0;
		}
	}
	return 0;
}

int _rx62t_FlashControl::flush(void)
{
	this->entry_flash_pe();
	page_head[this->page_size * this->page] = 0xE8;
	page_head[this->page_size * this->page] = 0x40;
	for(int i=0; i < this->page_size;i++){
		page_head[this->page_size * this->page] = buff[i];
	}
	page_head[this->page_size * this->page] = 0xd0;
	
	while(!FLASH.FSTATR0.BIT.FRDY);				//レディチェック
	
	this->entry_flash_normal();
	return 0;
}

int _rx62t_FlashControl::get_page(unsigned int page_num)
{
	if(page_num >= this->max_page_num())return -1;
	
	for(int i = 0;i < this->page_size;i++){
		this->buff[i] = page_head[i + page_size * page_num];	//ページめくりを加算表現。
	}
	return 0;
}

void _rx62t_FlashControl::entry_flash_pe(void)
{
	while(!FLASH.FSTATR0.BIT.FRDY);				//レディチェック
	FLASH.FENTRYR.WORD = 0xAA80;				//フラッシュP/Eモード
}

void _rx62t_FlashControl::entry_flash_normal(void)
{
	while(!FLASH.FSTATR0.BIT.FRDY);				//レディチェック
	FLASH.FENTRYR.WORD = 0xAA00;				//フラッシュP/Eモード
}