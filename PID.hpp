#ifndef _PID_HPP_
#define _PID_HPP_

template <typename type>
class PID{
	
private:
	float Kp;	//ゲイン
	float Ki;	//
	float Kd;	//
	
	float dt;	//微積分時間
	
	float sum;	//積和値
	float befor;	//前回偏差
	
	float Proportion;	//比例分の値(ゲイン調整時に使用)
	float Integration;	//積分分の値
	float Differentiation;	//
	
public:
	PID(float p,float i,float d,float t)
	{
		Kp = p;
		Ki = i;
		Kd = d;
		dt = t;
		sum = 0;
		befor = 0;
	}
	
	void SetPGain(float p)
	{
		Kp = p;
	}
	
	void SetIGain(float i)
	{
		Ki = i;
	}
	
	void SetDGain(float d)
	{
		Kd = d;
	}
	
	void SumReset(void)
	{
		sum = 0;
		befor = 0;
	}
	
	float Run(type data,type ref)
	{
		float error;
		float ret = 0;
		
		error = ref - data;
		
		sum += (error + befor) / 2 * dt;
		
		Proportion = Kp * error;
		Integration = Ki * sum;
		Differentiation = Kd * (error - befor) / dt;
		
		befor = error;
		ret = Proportion + Integration + Differentiation;
		return ret;
	}
};
/*/
template<typename type>
class PID{

private:
	float K;	//ゲイン
	float Ti;	//積分時間
	float Td;	//微分時間
	
	float dt;	//微積分時間（タイムスライス）
	
	float sum;	//積和値
	float befor;	//前回偏差
	
	float Proportion;	//比例分の値(ゲイン調整時に使用)
	float Integration;	//積分分の値
	float Differentiation;	//
	
	PID(float k,float i,float d,float t)
	{
		K = k;
		Ti = i;
		Td = d;
		dt = t;
		sum = 0;
		befor = 0;
	}
	
	void SetGain(float k){this->K = k;}
};
//*/
#endif