#ifndef _PID_HPP_
#define _PID_HPP_

template <typename type>
class PID{
	
private:
	float Kp;	//�Q�C��
	float Ki;	//
	float Kd;	//
	
	float dt;	//���ϕ�����
	
	float sum;	//�Ϙa�l
	float befor;	//�O��΍�
	
	float Proportion;	//��ᕪ�̒l(�Q�C���������Ɏg�p)
	float Integration;	//�ϕ����̒l
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
	float K;	//�Q�C��
	float Ti;	//�ϕ�����
	float Td;	//��������
	
	float dt;	//���ϕ����ԁi�^�C���X���C�X�j
	
	float sum;	//�Ϙa�l
	float befor;	//�O��΍�
	
	float Proportion;	//��ᕪ�̒l(�Q�C���������Ɏg�p)
	float Integration;	//�ϕ����̒l
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