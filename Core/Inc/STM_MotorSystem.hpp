/*
 * STM_MotorSystem.h
 *
 *  Created on: 2021/09/11
 *      Author: 0_hayate
 */
#ifndef INC_STM_MOTORSYSTEM_H_
#define INC_STM_MOTORSYSTEM_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include <CAN.hpp>
#include <PID.hpp>
#include <PWM.hpp>
#include <ENCODER.hpp>
#include <ADC.hpp>

typedef enum {
	VELOCITY_CONTROL = 0xFFFF,
	TORQUE_CONTROL = 0x0FFF,
	ANGLE_CONTROL = 0xF0FF,
	COAST_CONTROL = 0x00FF,
	SYSTEM_STOP = 0x0000
}mode;



class STM_MotorSystem
{
private:

	float velocity_ref; //エンコダからフィードバックされる速度
	float velocity_tar; //外部MCUから入力される目標速度[rad/sec]
	float velocity_buf; //外部MCUから入力される目標速度のバッファ(速度制御時以外で目標速度が変更されないように)
	float velocity_limit;

	float current_ref; //電流センサからフィードバックされる電流値
	float current_tar; //速度制御PIDから渡される目標電流値
	float current_buf; //外部MCUから入力される目標電流
	float current_limit;

	float angle_ref;
	float angle_tar;
	float angle_buf;

	float volt; //電源電圧

	float kt; //トルク係数 kv値[rpm/v]からだと 1/(rpm/v/60*2*3.141592)

	float ppr; //エンコダの分解能

	float velocity_p_buf; //pidゲインのバッファ
	float velocity_i_buf;
	float velocity_d_buf;

	float torque_p_buf;
	float torque_i_buf;
	float torque_d_buf;

	float angle_p_buf;
	float angle_i_buf;
	float angle_d_buf;

	float before_vel;

	uint32_t ofset_angle;
	float before_angle;

	GPIO_TypeDef *GPIO_dir;
	uint16_t GPIO_PIN_dir;

	GPIO_TypeDef *GPIO_coast;
	uint16_t GPIO_PIN_coast;

	uint8_t control_switch;

	uint32_t before_encoder_cnt;

	uint32_t MotorSystem_mode_buf;

	uint32_t MotorSystem_mode;

	TIM_HandleTypeDef *_control_timer;

	GPIO_PinState dir_f;

	uint8_t init_f; //二重初期化防止フラグ

public:
	STM_MotorSystem(ADC_HandleTypeDef *_hadc,
			CAN_HandleTypeDef *_hcan,
			TIM_HandleTypeDef *_encoder_timer,
			TIM_HandleTypeDef *_pwm_timer,
			uint32_t TIM_CHANNEL_n,
			TIM_HandleTypeDef *_control_timer
			); //コンストラクタ

	void STM_MotorSystem_init();

	void STM_MotorSystem_start();

	void STM_MotorSystem_stop();

	void set_velocity(float velocity_tar){ //通信系から目標速度をもらう
		this->velocity_buf = velocity_tar;
		this->MotorSystem_mode_buf = VELOCITY_CONTROL;
	}

	void set_torque(float torque_tar){
		torque_tar *=1/kt;
		this->current_buf = torque_tar;
		this->MotorSystem_mode_buf = TORQUE_CONTROL;
	}

	void set_angle(float angle_tar){
		this->angle_buf = angle_tar;
		this->MotorSystem_mode_buf = ANGLE_CONTROL;
	}

	float com_get_velocity(){//通信系に現在の速度を返す
		return this->velocity_ref;
	}

	float get_velocity(); //エンコダ出力から現在の速度を計算して返す



	float com_get_current(){//通信系に現在の電流を返す
		return current_ref;
	}
	float get_current(); //電流センサ出力から現在の電流を返す


	// int32_t get_angle_cnt();　ゴミ

	float get_angle(); //現在の角度を返す

	float get_sum_angle(); //エンコダから総角度変化を返す

	float com_get_sum_angle(){ //現在の角度
		return get_angle();
	}


	/*motor_control()でタイマー割りこみによるシステムを構成する上で、下記のようにコントローラ関数を命名すると分かりやい
	 *速度制御時にはcontller_velocity（）が追加され、トルク制御時にはcontroller_torque（）のみで構成できるため
	 */

	void controller_velocity(); /*速度制御を行う
	紛らわしいけどマイナーループを無視した全体の制御の入出力を見ると速度制御のように見えるからこの名前にした。
	（全体の関数名変えるのがだるい）
	この項では入力が速度で、出力として目標電流を吐き出す。
	厳密には制御周期で目標速度に到達するためのトルクをトルク定数で割った値を出力している。（PIDコントローラを用いたフィードバックシステムであるためロバストが存在するため書いたようにはならない）
	このロバストのおかげで制御対象の伝達関数が正確に分かっていなくても制御ができるわけだが

	制御対象のイナーシャが分かっていればFFも入れられるかもしれない。
	*/
	void controller_torque(); //トルク制御を行う
	/*
	 ここがモータに対して入力を行いトルクを制御している項であるため
	 上記の速度制御項が実質的にトルク目標値を出力しているが、こちらをトルク制御項とした
	 厳密には電流入力、電圧出力の制御項であり、モータの電圧トルク伝達関数に入力をあたえ、目標電流に一致するトルクを出力する電圧を出力している。
	 （PIDコントローラを用いたフィードバックシステムであるためロバストが存在するため書いたようにはならない）

	 */

	void controller_angle();

	void motor_control();

	void set_dir_pin(GPIO_TypeDef *GPIO_dir,uint16_t GPIO_PIN_dir); //dir用のピンを設定
	void set_dir(GPIO_PinState dir); //dirを設定

	void set_volt(float volt){
		this->volt = volt;
	}

	void set_kt(float kt){
		this->kt = kt;
	}

	void set_ppr(float ppr){
		this->ppr = ppr;
	}

	void set_current_limit(float current_limit){
		this->current_limit = current_limit;
	}

	void set_coast();

	void set_coast_pin(GPIO_TypeDef *GPIO_coast,uint16_t GPIO_PIN_coast){
		this->GPIO_PIN_coast = GPIO_PIN_coast;
		this->GPIO_coast = GPIO_coast;
	}


	void set_velocity_p(float velocity_p_buf){
		this->velocity_p_buf = velocity_p_buf;
	}

	void set_velocity_i(float velocity_i_buf){
		this->velocity_i_buf = velocity_i_buf;
	}

	void set_velocity_d(float velocity_d_buf){
		this->velocity_d_buf = velocity_d_buf;
	}



	void set_torque_p(float torque_p_buf){
		this->torque_p_buf = torque_p_buf;
	}

	void set_torque_i(float torque_i_buf){
		this->torque_i_buf = torque_i_buf;
	}

	void set_torque_d(float torque_d_buf){
		this->torque_d_buf = torque_d_buf;
	}



	void set_angle_p(float angle_p_buf){
		this->angle_p_buf = angle_p_buf;
	}

	void set_angle_i(float angle_i_buf){
		this->angle_i_buf = angle_i_buf;
	}

	void set_angle_d(float angle_d_buf){
		this->angle_d_buf = angle_d_buf;
	}



	void debug_func(){ //動作テスト用関数
		STM_MotorSystem_init();
		MotorSystem_mode_buf = 	VELOCITY_CONTROL;
		STM_MotorSystem_start();
	}

	void debug_func2(){
		  STM_MotorSystem_init();
		  set_torque_p(1);
		  set_torque_i(0);
		  set_torque_d(0.25);
		  set_kt(7.2*60/221/2/3.141592);
		  set_ppr(2048);
		  set_velocity_p(5.25);
		  set_velocity_i(0.0003);
		  set_velocity_d(0.33);
		  set_velocity(0.2*3.14);
		  //set_torque(-0.1);
		  STM_MotorSystem_start();
	}

	static STM_MotorSystem *_ms;



	PID pid_velocity;

	PID pid_torque;

	PID pid_angle;

	USER_CAN use_can;

	PWM use_pwm;

	ENCODER use_encoder;

	ADC use_adc;

};


inline void STM_MotorSystem::set_dir_pin(GPIO_TypeDef *GPIO_dir,uint16_t GPIO_PIN_dir){
	this->GPIO_PIN_dir = GPIO_PIN_dir;
	this->GPIO_dir = GPIO_dir;
}

inline void STM_MotorSystem::set_dir(GPIO_PinState dir){
	HAL_GPIO_WritePin (this->GPIO_dir,this->GPIO_PIN_dir,dir);
}

inline 	void STM_MotorSystem::set_coast(){
	this->MotorSystem_mode_buf = COAST_CONTROL;
	this->STM_MotorSystem_start();
}

inline void STM_MotorSystem::STM_MotorSystem_stop(){
	MotorSystem_mode_buf = SYSTEM_STOP;
	STM_MotorSystem_start();
}

#endif /* INC_STM_MOTORSYSTEM_H_ */
