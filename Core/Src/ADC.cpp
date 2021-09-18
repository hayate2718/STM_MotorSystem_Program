/*
 * ADC.cpp
 *
 *  Created on: 2021/09/14
 *      Author: 0_hayate
 */

#include <ADC.hpp>

ADC::ADC(ADC_HandleTypeDef *_hadc, float ADC_supply_voltage) :
		ofset_current(0), current(0), ADC_supply_voltage(ADC_supply_voltage), _hadc(
				_hadc) {
	_hadc->Init.Resolution = ADC_RESOLUTION_12B;
	switch (_hadc->Init.Resolution) {
	case ADC_RESOLUTION_12B:
		ADC_resolution = 2 ^ 12;
		break;

	case ADC_RESOLUTION_10B:
		ADC_resolution = 2 ^ 10;
		break;

	case ADC_RESOLUTION_8B:
		ADC_resolution = 2 ^ 8;
		break;

	case ADC_RESOLUTION_6B:
		ADC_resolution = 2 ^ 6;
		break;
	}

	__IO uint32_t *_isr_buf = &_hadc->Instance->ISR;
	_isr = (use_register*) _isr_buf;

	calibration_current[100] = { 0 };

	return;
}

void ADC::ADC_calibration() {
	HAL_ADC_Start(_hadc);
	_isr->bit2 = 1; //EOCbitクリア
	for (int i = 0; i < 100; i++) {
		while (!_isr->bit2)
			; //EOCbitが再セットされるのをまつ
		calibration_current[i] = _hadc->Instance->DR;
	}
	for (int j = 0; j < 100; j++) {
		ofset_current += calibration_current[j];
	}
	ofset_current /= 100;
	HAL_ADC_Stop(_hadc);
}

float ADC::ADC_get_current() { //電流センサ出力から現在の電流を計算して返す
	float real_current;

	_isr->bit2 = 1;
	while (!_isr->bit2);

	current = _hadc->Instance->DR - ofset_current;
	real_current = current * (ADC_supply_voltage / ADC_resolution);

	return real_current;

}
