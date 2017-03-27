#ifndef _queue_H_
#define _queue_H_

template <typename type>
class Queue{
	type *buff;
	unsigned int lenght;
	unsigned int head;
	unsigned int taill;
public:
	Queue(unsigned int lenght)
	{
		buff = malloc(sizeof(type) * lenght);
		this->lenght = lenght;
		head = 0;
		taill = 0;
	}
	type push(void)
	{
		type ret;
		ret = buff[head++];
		if(head == lenght)head = 0;
		return ret;
	}
	
	void pop(type data)
	{
		buff[taill++] = ret;
		if(taill == lenght)taill = 0;
	}
};

#endif