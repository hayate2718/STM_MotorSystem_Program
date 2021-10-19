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

volt(12),

kt(0),

ppr(0),

before_vel(0),

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

#ifndef debug
	//can id set
	use_can.GPIO_idbit0 = GPIOB;
	use_can.GPIO_idbit1 = GPIOB;
	use_can.GPIO_idbit2 = GPIOA;
	use_can.GPIO_idbit3 = GPIOA;
	use_can.GPIO_PIN_idbit0 = GPIO_PIN_1;
	use_can.GPIO_PIN_idbit1 = GPIO_PIN_8;
	use_can.GPIO_PIN_idbit2 = GPIO_PIN_9;
	use_can.GPIO_PIN_idbit3 = GPIO_PIN_10;
	use_can.filter_set();

	//can通信有効化
	HAL_CAN_Start(_hcan);
	HAL_CAN_ActivateNotification(_hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

#endif

	//pid init
	pid_velocity.PID_set_dt(0.001);
	pid_torque.PID_set_dt(0.0001);

	this->velocity_p_buf = 0;
	this->velocity_i_buf = 0;
	this->velocity_d_buf = 0;

	this->torque_p_buf = 0;
	this->torque_i_buf = 0;
	this->torque_d_buf = 0;

	//速度制御用エンコダバッファ初期化
	before_encoder_cnt = use_encoder.get_ofset();

	//速度、電流制限
	velocity_limit = 100; //ここはそこまで問題じゃない
	current_limit = 10; //こっちはちゃんと設定しないと積分がバグる。とくにストールとかさせたとき

	//a3921のdirピンの操作ピン設定
	set_dir_pin(GPIOB,GPIO_PIN_4);

	//coast機能ピン設定
	set_coast_pin(GPIOA,GPIO_PIN_7);

	//電流センサゲインセット
	this->use_adc.ADC_set_gain(0.025);


}

void STM_MotorSystem::STM_MotorSystem_init(){
	HAL_TIM_Base_Stop_IT(_control_timer); //割り込みタイマ停止
	_control_timer->Instance->CNT = 0; //割り込みタイマカウント初期化
	this->control_switch = 0;

	this->use_adc.ADC_calibration(); //adcのキャリブレーション

	this->use_encoder.init_ENCODER(); //エンコダカウント初期化

	this->use_pwm.PWM_stop(); //PWMdutyを0にする

	HAL_GPIO_WritePin(this->GPIO_coast,this->GPIO_PIN_coast,GPIO_PIN_RESET); //coast無効化

	this->MotorSystem_mode_buf = SYSTEM_STOP; //システムをストップモードにセット
}


void STM_MotorSystem::STM_MotorSystem_start(){ //スタート毎にモードの初期化が行われる
	HAL_TIM_Base_Stop_IT(_control_timer);
	_control_timer->Instance->CNT = 0; //割り込みタイマカウント初期化
	__HAL_TIM_CLEAR_FLAG(_control_timer, TIM_FLAG_UPDATE);
	this->control_switch = 0;

	HAL_GPIO_WritePin(this->GPIO_coast,this->GPIO_PIN_coast,GPIO_PIN_RESET);

	switch(MotorSystem_mode_buf){
	case VELOCITY_CONTROL:
		this->MotorSystem_mode = VELOCITY_CONTROL;

		pid_velocity.PID_set_p(velocity_p_buf); //pid gain set
		pid_velocity.PID_set_i(velocity_i_buf);
		pid_velocity.PID_set_d(velocity_d_buf);

		pid_torque.PID_set_p(torque_p_buf);
		pid_torque.PID_set_i(torque_i_buf);
		pid_torque.PID_set_d(torque_d_buf);

		this->use_adc.ADC_start();
		HAL_TIM_Base_Start_IT(_control_timer);
		break;

	case TORQUE_CONTROL:
		this->MotorSystem_mode = TORQUE_CONTROL;

		pid_torque.PID_set_p(torque_p_buf);
		pid_torque.PID_set_i(torque_i_buf);
		pid_torque.PID_set_d(torque_d_buf);

		this->velocity_tar = 0; //トルクコントロールモードでのフィードフォワード無効化

		this->use_adc.ADC_start();
		HAL_TIM_Base_Start_IT(_control_timer);
		break;

	case SYSTEM_STOP:
		this->MotorSystem_mode = SYSTEM_STOP;
		this->use_pwm.PWM_stop();
		this->use_adc.ADC_stop();
		break;

	case COAST_CONTROL:
		this->MotorSystem_mode = COAST_CONTROL;
		this->use_pwm.PWM_stop();
		HAL_GPIO_WritePin(this->GPIO_coast,this->GPIO_PIN_coast,GPIO_PIN_SET);
		this->use_adc.ADC_stop();
		break;
	}
}

STM_MotorSystem * STM_MotorSystem::_ms = 0;
