/*
 * STM_MotorSystem_init.cpp
 *
 *  Created on: Sep 11, 2021
 *      Author: 0_hayate
 */

#include <STM_MotorSystem.hpp>

STM_MotorSystem::STM_MotorSystem(ADC_HandleTypeDef *_hadc,
		CAN_HandleTypeDef *_hcan,
		TIM_HandleTypeDef *_encoder_timer,
		TIM_HandleTypeDef *_pwm_timer,
		uint32_t TIM_CHANNEL_n
		):
pid_velocity(0,0,0,0.0001),
pid_torque(0,0,0,0.0001),
use_can(_hcan),
use_pwm(_pwm_timer,TIM_CHANNEL_n),
use_encoder(_encoder_timer),
velocity_ref(0),
velocity_tar(0),
current_ref(0),
volt(0)
{
	//can id set
	use_can.GPIO_idbit0 = GPIOB;
	use_can.GPIO_idbit1 = GPIOB;
	use_can.GPIO_idbit2 = GPIOA;
	use_can.GPIO_idbit3 = GPIOA;
	use_can.GPIO_PIN_idbit0 = GPIO_PIN_1;
	use_can.GPIO_PIN_idbit1 = GPIO_PIN_8;
	use_can.GPIO_PIN_idbit2 = GPIO_PIN_9;
	use_can.GPIO_PIN_idbit3 = GPIO_PIN_10;


}

