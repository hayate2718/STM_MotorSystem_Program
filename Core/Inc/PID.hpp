/*
 * PID.h
 *
 *  Created on: 2021/09/07
 *      Author: 0_hayate
 */

#ifndef INC_PID_HPP_
#define INC_PID_HPP_
#include "main.h"

#include <math.h>

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
	float p_mv;
	float i_mv;
	float d_mv;

	float i_lim; //積分リミッタ
	float mv_lim; //制御量リミッタ

public:
	PID(float p,float i,float d,float dt,float i_lim,float mv_lim);
	void PID_set_p(float p);
	void PID_set_i(float i);
	void PID_set_d(float d);
	void PID_set_dt(float dt);

	void PID_set_i_lim(float lim);
	void PID_set_mv_lim(float lim);

	float PID_get_p();
	float PID_get_i();
	float PID_get_d();

	float PID_controller(float error);

	void PID_reset();
};

inline PID::PID(float p,float i,float d,float dt,float i_lim,float mv_lim):
		p(p),
		i(i),
		d(d),
		dt(dt),
		error(0),
		i_sum(0),
		error_before(0),
		p_mv(0),
		i_mv(0),
		d_mv(0),
		i_lim(i_lim),
		mv_lim(mv_lim)
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

inline void PID::PID_set_i_lim(float lim){
	this->i_lim = lim;
}

inline void PID::PID_set_mv_lim(float lim){
	this->mv_lim = lim;
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
	float MV = 0; //PIDコントローラ操作量

	i_sum = i_sum + dt*(error+error_before)/2; //微小時間の間線形に動いていたとして

	if(fabsf(i_sum) > this->i_lim){
		if(i_sum > 0){
			i_sum = i_lim;
		}else{
			i_sum = -1*i_lim;
		}
	}

	p_mv = this->p*error;
	i_mv = this->i*i_sum;
	d_mv = this->d*(error-error_before)/dt;

	MV = p_mv+i_mv+d_mv;

	if(fabsf(MV) > this->mv_lim){
		if(MV > 0){
			MV = mv_lim;
		}else{
			MV = -1*mv_lim;
		}
	}

	error_before = error;

	return MV;
}

inline void PID::PID_reset(){
	i_sum = 0;
	error_before = 0;
}

#endif /* INC_PID_HPP_ */
