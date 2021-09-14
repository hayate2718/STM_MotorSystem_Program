/*
 * TIM3.hpp
 *
 *  Created on: 2021/09/11
 *      Author: 0_hayate
 */

#ifndef INC_PWM_HPP_
#define INC_PWM_HPP_

#include "main.h"

class PWM{
private:
	float supply_voltage; //電源電圧

	TIM_HandleTypeDef *_pwm_timer;

	__IO uint32_t * CCRn;
public:
	PWM(TIM_HandleTypeDef *_pwm_timer, uint32_t TIM_CHANNEL_n ); //TIMxCHn n=1,2,3...

	void PWM_start(float voltage);
	void PWM_stop();

};



#endif /* INC_PWM_HPP_ */
