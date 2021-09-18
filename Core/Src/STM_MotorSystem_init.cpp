/*
 * STM_MotorSystem_init.cpp
 *
 *  Created on: Sep 11, 2021
 *      Author: 0_hayate
 */

#include <STM_MotorSystem.hpp>

STM_MotorSystem::STM_MotorSystem(
		ADC_HandleTypeDef *_hadc,
		CAN_HandleTypeDef *_hcan,
		TIM_HandleTypeDef *_encoder_timer,
		TIM_HandleTypeDef *_pwm_timer,
		uint32_t TIM_CHANNEL_n,
		TIM_HandleTypeDef *_control_timer
		):
velocity_ref(0),
velocity_tar(0),

current_ref(0),
current_tar(0),

volt(0),

kt(0),

ppr(0),

control_switch(0),

pid_velocity(0,0,0,0.0001),
pid_torque(0,0,0,0.0001),
use_can(_hcan),
use_pwm(_pwm_timer,TIM_CHANNEL_n),
use_encoder(_encoder_timer),
use_adc(_hadc,3.3)

{

	_ms = this;

	this->_control_timer = _control_timer;

	//can id set
	use_can.GPIO_idbit0 = GPIOB;
	use_can.GPIO_idbit1 = GPIOB;
	use_can.GPIO_idbit2 = GPIOA;
	use_can.GPIO_idbit3 = GPIOA;
	use_can.GPIO_PIN_idbit0 = GPIO_PIN_1;
	use_can.GPIO_PIN_idbit1 = GPIO_PIN_8;
	use_can.GPIO_PIN_idbit2 = GPIO_PIN_9;
	use_can.GPIO_PIN_idbit3 = GPIO_PIN_10;

	//can通信有効化
	HAL_CAN_Start(_hcan);
	HAL_CAN_ActivateNotification(_hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

	//pid dt set
	pid_velocity.PID_set_dt(0.001);
	pid_torque.PID_set_dt(0.0001);

	//速度制御用エンコダバッファ初期化
	before_encoder_cnt = use_encoder.get_ofset();

	//速度、電流制限
	velocity_limit = 100; //ここはそこまで問題じゃない
	current_limit = 10; //こっちはちゃんと設定しないと積分がバグる。とくにストールとかさせたとき

	//a3921のdirピンの操作ピン設定
	set_dir_pin(GPIOB,GPIO_PIN_4);

	//coast機能ピン設定
	set_coast_pin(GPIOA,GPIO_PIN_7);

}

void STM_MotorSystem::STM_MotorSystem_init(){
	HAL_TIM_Base_Stop_IT(_control_timer);
	this->use_adc.ADC_calibration();
	this->use_encoder.init_ENCODER();
	this->use_pwm.PWM_stop();
	HAL_GPIO_WritePin(this->GPIO_coast,this->GPIO_PIN_coast,GPIO_PIN_RESET);
	this->MotorSystem_mode_buf = SYSTEM_STOP;
}

void STM_MotorSystem::STM_MotorSystem_start(){

	HAL_GPIO_WritePin(this->GPIO_coast,this->GPIO_PIN_coast,GPIO_PIN_RESET);
	__HAL_TIM_CLEAR_FLAG(_control_timer, TIM_FLAG_UPDATE);

	switch(MotorSystem_mode_buf){
	case VELOCITY_CONTROL:
		this->MotorSystem_mode = VELOCITY_CONTROL;
		this->control_switch = 0;
		this->use_adc.ADC_start();
		HAL_TIM_Base_Start_IT(_control_timer);
		break;

	case TORQUE_CONTROL:
		this->MotorSystem_mode = TORQUE_CONTROL;
		this->control_switch = 0;
		this->use_adc.ADC_start();
		HAL_TIM_Base_Start_IT(_control_timer);
		break;

	case SYSTEM_STOP:
		this->MotorSystem_mode = SYSTEM_STOP;
		this->control_switch = 0;
		HAL_TIM_Base_Stop_IT(_control_timer);
		this->use_pwm.PWM_stop();
		//this->use_adc.ADC_stop();
		break;

	case COAST_CONTROL:
		this->MotorSystem_mode = COAST_CONTROL;
		this->control_switch = 0;
		HAL_TIM_Base_Stop_IT(_control_timer);
		this->use_pwm.PWM_stop();
		HAL_GPIO_WritePin(this->GPIO_coast,this->GPIO_PIN_coast,GPIO_PIN_SET);
		//this->use_adc.ADC_stop();
		break;
	}
}

STM_MotorSystem * STM_MotorSystem::_ms = 0;
