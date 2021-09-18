/*
 * PWM.cpp
 *
 *  Created on: Sep 13, 2021
 *      Author: 0_hayate
 */


#include <PWM.hpp>

PWM::PWM(TIM_HandleTypeDef *_pwm_timer,uint32_t TIM_CHANNEL_n ){ //TIMxCHn n=1,2,3...
	this->_pwm_timer = _pwm_timer;

	switch(TIM_CHANNEL_n){
	case TIM_CHANNEL_1:
		CCRn = & this->_pwm_timer->Instance->CCR1;
		*CCRn = 0;
		break;

	case TIM_CHANNEL_2:
		CCRn = & this->_pwm_timer->Instance->CCR2;
		*CCRn = 0;
		break;

	case TIM_CHANNEL_3:
		CCRn = & this->_pwm_timer->Instance->CCR3;
		*CCRn = 0;
		break;

	case TIM_CHANNEL_4:
		CCRn = & this->_pwm_timer->Instance->CCR4;
		*CCRn = 0;
		break;

	default:
		break;

	}

	HAL_TIM_PWM_Start(_pwm_timer, TIM_CHANNEL_n);

	this->arr = this->_pwm_timer->Instance->ARR;

}

void PWM::PWM_out(float voltage){

	uint32_t buf;

	buf = arr / supply_voltage * voltage;

	if(buf > arr){ //pwmdutyリミット
		buf = arr;
	}

	*CCRn = buf;
}

void PWM::PWM_stop(){ //pwmタイマ自体は動作している。dutyを0にしているだけ
	*CCRn = 0;
}
