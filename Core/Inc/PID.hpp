/*
 * PID.h
 *
 *  Created on: 2021/09/07
 *      Author: 0_hayate
 */

#ifndef INC_PID_HPP_
#define INC_PID_HPP_
#include "main.h"

class PID
{
private:
	float p; //比例ゲイン
	float i; //積分ゲイン
	float d; //微分ゲイン
	float dt; //微小時間（制御周期）
	float error; //偏差
	float i_sum; //積分制御用偏差バッファ
	float error_before; //前回偏差

public:
	PID(float p,float i,float d,float dt);
	void PID_set_p(float p);
	void PID_set_i(float i);
	void PID_set_d(float d);
	void PID_set_dt(float dt);
	float PID_get_p();
	float PID_get_i();
	float PID_get_d();
	float PID_controller(float error);
};

inline PID::PID(float p,float i,float d,float dt):
		p(p),i(i),d(d),dt(dt),error(0),i_sum(0),error_before(0)
{
	return;
}

inline void PID::PID_set_p(float p){
	this->p = p;
}

inline void PID::PID_set_i(float i){
	this->i = i;
}

inline void PID::PID_set_d(float d){
	this->d = d;
}

inline void PID::PID_set_dt(float dt){
	this->dt = dt;
}

inline float PID::PID_get_p(){
	float p;
	p = this->p;
	return p;
}

inline float PID::PID_get_i(){
	float i;
	i = this->i;
	return i;
}

inline float PID::PID_get_d(){
	float d;
	d = this->d;
	return d;
}

inline float PID::PID_controller(float error){
	this->error = error;
	float MV = 0; //PIDコントローラ操作量
	float p_mv = 0;
	float i_mv = 0;
	float d_mv = 0;

	this->i_sum = (this->error+this->error_before)/2; //微小時間の間線形に動いていたとして

	p_mv = this->p*this->error;
	i_mv = this->i*i_sum*this->dt;
	d_mv = this->d*(this->error-error_before)/dt;

	MV = p_mv+i_mv+d_mv;
	error_before = this->error;

	return MV;
}

#endif /* INC_PID_HPP_ */
