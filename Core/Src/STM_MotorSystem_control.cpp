/*
 * STM_MotorSystem_control.cpp
 *
 *  Created on: 2021/09/18
 *      Author: 0_hayate
 */

#include <STM_MotorSystem.hpp>

void STM_MotorSystem::motor_control(){
	/*
	 * defaultには本来ありえないパターンの処理をおいている。すなはち、例外発生であるためモータシステムが停止する処理に入る。
	 * switch文でcontrol_switch変数を用いてタイマー割り込みの回数によって処理を変化させている。（10kHzで割り込みが入る）
	 * 機能追加の際はMotorSystem_start()内に初期化文を書き、タイマー割り込み周期ごとの処理をここに書く。
	 * 割り込み周期ごとの処理書く際、重複する処理はフォースルーで収束させる。
	 * 制御周期を長期に変化させたい場合は各caseにbreakを記述し、特定のcaseに達したら処理を行わせcontrol_switchを初期化する。
	 * 重複する処理や、制御周期を長期にしたい場合defaultに処理をおくか、breakさせるのがラクだが例外判定ができないため必ずdefaultには例外処理をおく(システム停止でなくてもよい)。
	 */
	control_switch++;
	switch(this->MotorSystem_mode){
	case VELOCITY_CONTROL:
		{switch(control_switch){
		case 1:
			this->controller_velocity();
			this->controller_torque();
			break;

		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
			this->controller_torque();
			break;

		case 10:
			this->controller_torque();
			control_switch = 0;
			break;

		default :
			this->MotorSystem_mode_buf = SYSTEM_STOP;
			this->STM_MotorSystem_start();
			break;

		}}break;

	case TORQUE_CONTROL:
		this->current_tar = this->current_buf;
		{switch(control_switch){
		case 1:
			this->controller_torque();
			control_switch = 0;
			break;

		default:
			this->MotorSystem_mode_buf = SYSTEM_STOP;
			this->STM_MotorSystem_start();
			break;

		}}break;

	case COAST_CONTROL:
		HAL_TIM_Base_Stop_IT(_control_timer);
		this->use_pwm.PWM_stop();
		HAL_GPIO_WritePin(this->GPIO_coast,this->GPIO_PIN_coast,GPIO_PIN_SET);
		this->use_adc.ADC_stop();
		break;

	default:
		HAL_TIM_Base_Stop_IT(_control_timer);
		this->use_pwm.PWM_stop();
		this->use_adc.ADC_stop();
		break;

	}
}




void STM_MotorSystem::controller_velocity(){
	this->velocity_ref = this->get_velocity();
	float e_velocity;
	this->velocity_tar = this->velocity_buf;

	if(fabsf(velocity_tar) > velocity_limit){
			if(velocity_tar > 0){
				velocity_tar = velocity_limit;
			}else{
				velocity_tar = -1*velocity_limit;
			}
		}

	e_velocity = this->velocity_tar - this->velocity_ref;

	current_tar = this->pid_velocity.PID_controller(e_velocity);

	return;
}



void STM_MotorSystem::controller_torque(){
	float e_current;
	float volt_tar;

	if(fabsf(current_tar) > current_limit){
				if(current_tar > 0){
					current_tar = current_limit;
				}else{
					current_tar = -1*current_limit;
				}
			}

	e_current = current_tar - this->get_current();

	volt_tar = this->pid_torque.PID_controller(e_current);
	volt_tar += velocity_tar*kt + this->velocity_ref*kt; //フィードフォワードとフィードバックをたす

	if(volt_tar >= 0){ //モータの回転方向を決める
			dir_f = GPIO_PIN_RESET;
		}else{
			dir_f = GPIO_PIN_SET;
		}

	this->use_pwm.PWM_out(fabsf(volt_tar));
	set_dir(dir_f);

	return;
}



float STM_MotorSystem::get_velocity(){
	int64_t buf;
	float velocity;

	buf = this->use_encoder.get_count();
	buf -= before_encoder_cnt;
	before_encoder_cnt += buf;
	velocity = buf;
	velocity *=1570.796/ppr;

	velocity = velocity*0.7+0.3*before_vel;
	before_vel = velocity;

	return velocity;
}


float STM_MotorSystem::get_current(){
	this->current_ref = use_adc.ADC_get_current();
	return current_ref;
}


void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim){ //tim1割り込みコールバック

	STM_MotorSystem * ms = STM_MotorSystem::_ms;
	ms->motor_control();
	return;
}
