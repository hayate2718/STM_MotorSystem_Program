/*
 * TIM2.hpp
 *
 *  Created on: 2021/09/11
 *      Author: 0_hayate
 */

#ifndef INC_ENCODER_HPP_
#define INC_ENCODER_HPP_

#include "main.h"

class ENCODER{
private :
	uint32_t ENCODER_count;
	uint32_t ofset_count;
	TIM_HandleTypeDef *_encoder_timer;

public :
	ENCODER(TIM_HandleTypeDef *_encoder_timer);

	uint32_t get_count();

	uint32_t get_ofset();

	void init_ENCODER();

};

inline uint32_t ENCODER::get_count(){
	ENCODER_count = _encoder_timer->Instance->CNT;
	return ENCODER_count;
}



#endif /* INC_ENCODER_HPP_ */
