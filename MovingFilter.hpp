#ifndef _MovingFilter_HPP_
#define _MovingFilter_HPP_

template<typename type>
class MovingFilter{
	type data[30];
	int num;
public:
	MovingFilter(int _num)
	{
		this->num = _num;
		for(int i=0;i < 10;i++){
			data[i] = 0;
		}
	}
	
	float Put(type d)
	{
		float sum=0;
		for(int i=0;i < (num - 1);i++){
			sum += data[i];
		}
		sum += d;
		for(int j =0;j < num;j++){
			data[j + 1] = data[j];
		}
		data[0] = d;
		return sum / num;
	}	
};

template<typename type>
class IIR_Filter{
	type data[30];
	float K;
	int num;
public:
	IIR_Filter(float _K,int _num = 1)
	{
		this->K = _K;
		this->num = _num;
		for(int i=0;i < num;i++){
			data[i] = 0;
		}
	}
	
	float Put(type d)
	{
		float ret;
		ret = d * K + (1-K)*data[0];
		data[0] = ret;
		return ret;
	}	
};

#endif