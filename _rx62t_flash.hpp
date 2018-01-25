#ifndef _rx62t_flash_HPP_
#define _rx62t_flash_HPP_

class _rx62t_FlashControl{
	long count;
	long page;
	unsigned char buff[128];
	
	unsigned char *page_head;
	static const int page_size;
	static const int flash_area_size;
public:
	_rx62t_FlashControl(void);
	int read(char *,unsigned long);
	int write(const char *,unsigned long);
	int flush(void);
	int seek(unsigned long);
	
private:
	int max_page_num(void){return flash_area_size / page_size;}
	int get_page(unsigned int page_num);
	
	void entry_flash_pe(void);
	void entry_flash_normal(void);
};

#endif