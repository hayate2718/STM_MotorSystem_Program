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
#ifndef ccr
#define ccr this->_pwm_timer->Instance->CCR1
#endif
		ccr = 0;
		break;

	case TIM_CHANNEL_2:
#ifndef ccr
#define ccr this->_pwm_timer->Instance->CCR2
#endif
		ccr = 0;
		break;

	case TIM_CHANNEL_3:
#ifndef ccr
#define ccr this->_pwm_timer->Instance->CCR3
#endif
		ccr = 0;
		break;

	case TIM_CHANNEL_4:
#ifndef ccr
#define ccr this->_pwm_timer->Instance->CCR4
#endif
		ccr = 0;
		break;

	default:
		break;

	}

	HAL_TIM_PWM_Start(_pwm_timer, TIM_CHANNEL_n);
}

void PWM::PWM_start(float voltage){
	uint32_t arr;
	uint32_t buf;

	arr = this->_pwm_timer->Instance->ARR;
	buf = arr / supply_voltage * voltage;

	if(buf > arr){
		buf = arr;
	}

	ccr = buf;
}

void PWM::PWM_stop(){
	ccr = 0;
}
