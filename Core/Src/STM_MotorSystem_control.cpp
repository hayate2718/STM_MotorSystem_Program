/*
 * STM_MotorSystem_control.cpp
 *
 *  Created on: 2021/09/18
 *      Author: 0_hayate
 */

#include <STM_MotorSystem.hpp>

void STM_MotorSystem::motor_control(){
	control_switch++;
	switch(this->MotorSystem_mode){
	case VELOCITY_CONTROL:
		{switch(control_switch){
		case 1:
			this->controller_velocity();
			this->controller_torque();
			break;
		case 2:
			this->controller_torque();
			break;
		case 3:
			this->controller_torque();
			break;
		case 4:
			this->controller_torque();
			control_switch = 0;
			break;
		}}break;

	case TORQUE_CONTROL:
		this->current_tar = this->current_buf;
		{switch(control_switch){
		case 1:
			this->controller_torque();
			break;
		case 2:
			this->controller_torque();
			break;
		case 3:
			this->controller_torque();
			break;
		case 4:
			this->controller_torque();
			control_switch = 0;
			break;
		}}break;


	default:
		control_switch = 0;
			break;

	}
}




void STM_MotorSystem::controller_velocity(){
	float e_velocity;
	this->velocity_tar = this->velocity_buf;

	if(fabsf(velocity_tar) > velocity_limit){
			if(velocity_tar > 0){
				velocity_tar = velocity_limit;
			}else{
				velocity_tar = -1*velocity_limit;
			}
		}

	this->velocity_ref = this->get_velocity();

	e_velocity = this->velocity_tar - this->velocity_ref;

	current_tar = this->pid_velocity.PID_controller(e_velocity);

	if(fabsf(current_tar) > current_limit){
			if(current_tar > 0){
				current_tar = current_limit;
			}else{
				current_tar = -1*current_limit;
			}
		}

	return;
}



void STM_MotorSystem::controller_torque(){
	float e_current;
	float volt_tar;
	GPIO_PinState dir_f;

	e_current = this->get_current() - current_tar;

	volt_tar = this->pid_torque.PID_controller(fabsf(e_current));
	volt_tar += velocity_tar*kt+this->velocity_ref*kt; //フィードフォワードとフィードバックをたす

	this->use_pwm.PWM_out(fabsf(volt_tar));

	if(e_current > 0){ //モータの回転方向を決める
		dir_f = GPIO_PIN_SET;
	}else{
		dir_f = GPIO_PIN_RESET;
	}

	set_dir(dir_f);

	return;
}



float STM_MotorSystem::get_velocity(){
	int64_t buf;
	float velocity;

	buf = this->use_encoder.get_count() - before_encoder_cnt;
	buf *=2*3.141592/ppr;
	velocity = buf/0.001;

	return velocity;
}



float STM_MotorSystem::get_current(){
	this->current_ref = use_adc.ADC_get_current();
	return current_ref;
}



void STM_MotorSystem::set_dir_pin(GPIO_TypeDef *GPIO_dir,uint16_t GPIO_PIN_dir){
	this->GPIO_PIN_dir = GPIO_PIN_dir;
	this->GPIO_dir = GPIO_dir;
}



void STM_MotorSystem::set_dir(GPIO_PinState dir){
	HAL_GPIO_WritePin (this->GPIO_dir,this->GPIO_PIN_dir,dir);
}


void STM_MotorSystem::set_coast(){
	this->MotorSystem_mode_buf = COAST_CONTROL;
	this->STM_MotorSystem_start();

}

void STM_MotorSystem::set_coast_pin(GPIO_TypeDef *GPIO_coast,uint16_t GPIO_PIN_coast){
	this->GPIO_PIN_coast = GPIO_PIN_coast;
	this->GPIO_coast = GPIO_coast;
}


void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim){ //tim1割り込みコールバック
	STM_MotorSystem * ms = STM_MotorSystem::_ms;
	ms->motor_control();
	return;
}
