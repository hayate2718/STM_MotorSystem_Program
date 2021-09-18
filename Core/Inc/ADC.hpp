/*
 * ADC.hpp
 *
 *  Created on: 2021/09/11
 *      Author: 0_hayate
 */

#ifndef INC_ADC_HPP_
#define INC_ADC_HPP_

#include "main.h"

typedef union{
	struct{
		uint8_t bit0 : 1 ;
		uint8_t bit1 : 1 ;
		uint8_t bit2 : 1 ;
		uint8_t bit3 : 1 ;
		uint8_t bit4 : 1 ;
		uint8_t bit5 : 1 ;
		uint8_t bit6 : 1 ;
		uint8_t bit7 : 1 ;
		uint8_t bit8 : 1 ;
		uint8_t bit9 : 1 ;
		uint8_t bit10 : 1 ;
		uint8_t bit11 : 1 ;
		uint8_t bit12 : 1 ;
		uint8_t bit13 : 1 ;
		uint8_t bit14 : 1 ;
		uint8_t bit15 : 1 ;
		uint8_t bit16 : 1 ;
		uint8_t bit17 : 1 ;
		uint8_t bit18 : 1 ;
		uint8_t bit19 : 1 ;
		uint8_t bit20 : 1 ;
		uint8_t bit21 : 1 ;
		uint8_t bit22 : 1 ;
		uint8_t bit23 : 1 ;
		uint8_t bit24 : 1 ;
		uint8_t bit25 : 1 ;
		uint8_t bit26 : 1 ;
		uint8_t bit27 : 1 ;
		uint8_t bit28 : 1 ;
		uint8_t bit29 : 1 ;
		uint8_t bit30 : 1 ;
		uint8_t bit31 : 1 ;
	};
	__IO uint32_t all_bits;
} use_register;

class ADC{
private:
	uint16_t calibration_current[100];
	uint32_t ofset_current;
	int32_t current;
	//uint16_t buf_current[100];

	uint32_t ADC_resolution;

	float ADC_supply_voltage;

	ADC_HandleTypeDef *_hadc;

	use_register *_isr;
	use_register *_cr;

public:
	ADC(ADC_HandleTypeDef *_hadc,float ADC_supply_voltage);

	void ADC_calibration();

	//void ADC_current_fillter(); デジタルフィルタについて学習後実装予定

	float ADC_get_current();

	void ADC_start();

	void ADC_stop();//ADCをストップさせると復帰できるようになったが不安定

};




#endif /* INC_ADC_HPP_ */
