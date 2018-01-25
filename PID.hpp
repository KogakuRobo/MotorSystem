#ifndef _PID_HPP_
#define _PID_HPP_

/*/
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
//*/

//*/
template<typename type>
class PID{

private:
public:
	float K;	//ゲイン
	float Ti;	//積分時間
	float Td;	//微分時間
	
	float dt;	//微積分時間（タイムスライス）
	
	float sum;	//積和値
	float befor;	//前回偏差
	
	float Proportion;	//比例分の値(ゲイン調整時に使用)
	float Integration;	//積分分の値
	float Differentiation;	//
	
	bool sum_limit_flag;
	float sum_limit;
public:
	PID(float k,float i,float d,float t)
	{
		K = k;
		Ti = i;
		Td = d;
		dt = t;
		sum = 0;
		befor = 0;
		
		sum_limit_flag = false;
	}
	
	void SetK(float k){this->K = k;}
	
	void SetTi(float ti){this->Ti = ti;}
	
	void SetTd(float td){this->Td = td;}
	
	void Setdt(float _dt){this->dt = _dt;}
	
	void SumReset(void)
	{
		sum = 0;
		befor = 0;
	}
	
	void SetSumLimit(float _sum_)
	{
		sum_limit_flag = true;
		sum_limit = _sum_;
	}
	
	void SetIntegrationLimit(float _inte_)
	{
		SetSumLimit(_inte_ * this.Ti / K);
	}
	
	float Run(type data,type ref)
	{
		float error;
		float ret = 0;
		
		error = ref - data;
		
		float s_ep = (error + befor) / 2 * dt;	//今回の加算分
		
		//フラグが真でかつ、合計更新後の絶対値がリミットより大きい場合、更新はしない。　の~
		if( (abs(sum + s_ep) < sum_limit) || (!sum_limit_flag))
			sum += s_ep;
		
		Proportion = K * error;
		
		if(Ti > 0.0)Integration = K * sum / Ti;
		else Integration = 0;
		
		Differentiation = K * Td * (error - befor) / dt;
		
		befor = error;
		ret = Proportion + Integration + Differentiation;
		return ret;
	}
};
//*/
#endif