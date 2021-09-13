/*
 * ENCODER.cpp
 *
 *  Created on: 2021/09/13
 *      Author: 0_hayate
 */

#include <ENCODER.hpp>

ENCODER::ENCODER(TIM_HandleTypeDef *_encoder_timer)
{
	this->_encoder_timer = _encoder_timer;
	uint32_t arr;
	arr = this->_encoder_timer->Instance->ARR; //timerカウントの最大値
	this->_encoder_timer->Instance->CNT = arr/2;
	ENCODER_count = arr/2;

	HAL_TIM_Encoder_Start(this->_encoder_timer,TIM_CHANNEL_ALL);

	return;
}

uint32_t ENCODER::get_count(){
	ENCODER_count = _encoder_timer->Instance->CNT;
	return ENCODER_count;
}



