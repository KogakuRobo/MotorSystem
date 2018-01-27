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

const int _rx62t_FlashControl::page_size = 0x80;			//��y�[�W������128�o�C�g
const int _rx62t_FlashControl::flash_area_size = 0x0800 * 4;	//flash�S�̂�2K * 4�o�C�g

_rx62t_FlashControl::_rx62t_FlashControl(void)
{
	FLASH.DFLRE0.WORD = 0x2D0F;			//�Ǐo������
	FLASH.DFLWE0.WORD = 0x1E0F;			//�������݋���
	page_head = (unsigned char *)0x00100000;
	FLASH.PCKAR.WORD = PCK_CLOCK;
	this->entry_flash_pe();
	{	//�N���b�N�ʒm
		page_head[0] = 0xE9;
		page_head[0] = 0x03;
		((unsigned short *)page_head)[0] = 0x0F0F;
		((unsigned short *)page_head)[0] = 0x0F0F;
		((unsigned short *)page_head)[0] = 0x0F0F;
		page_head[0] = 0xD0;
		while(!FLASH.FSTATR0.BIT.FRDY);				//���f�B�`�F�b�N
	}
	this->entry_flash_normal();
	get_page(0);
	
	count = 0;					//�Ǐo�������l0
	page = 0;					//�Ǐo�������l0
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
	
	while(!FLASH.FSTATR0.BIT.FRDY);				//���f�B�`�F�b�N
	
	this->entry_flash_normal();
	return 0;
}

int _rx62t_FlashControl::get_page(unsigned int page_num)
{
	if(page_num >= this->max_page_num())return -1;
	
	for(int i = 0;i < this->page_size;i++){
		this->buff[i] = page_head[i + page_size * page_num];	//�y�[�W�߂�������Z�\���B
	}
	return 0;
}

void _rx62t_FlashControl::entry_flash_pe(void)
{
	while(!FLASH.FSTATR0.BIT.FRDY);				//���f�B�`�F�b�N
	FLASH.FENTRYR.WORD = 0xAA80;				//�t���b�V��P/E���[�h
}

void _rx62t_FlashControl::entry_flash_normal(void)
{
	while(!FLASH.FSTATR0.BIT.FRDY);				//���f�B�`�F�b�N
	FLASH.FENTRYR.WORD = 0xAA00;				//�t���b�V��P/E���[�h
}