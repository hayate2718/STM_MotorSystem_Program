/*
 * STM_MotorSystem.h
 *
 *  Created on: 2021/09/11
 *      Author: 0_hayate
 */
#ifndef INC_STM_MOTORSYSTEM_H_
#define INC_STM_MOTORSYSTEM_H_

#include <stdio.h>
#include <CAN.hpp>
#include <PID.hpp>
#include "main.h"


class STM_MotorSystem
{
private:
	float velocity_ref; //エンコダからフィードバックされる速度
	float velocity_tar; //外部MCUから入力される目標速度

	float current_ref; //電流センサからフィードバックされる電流値

	float volt; //電源電圧

public:
	STM_MotorSystem(ADC_HandleTypeDef *_hadc,
			CAN_HandleTypeDef *_hcan,
			TIM_HandleTypeDef *_encoder_timer,
			TIM_HandleTypeDef *_pwm_timer
			); //コンストラクタ

	void set_velocity(float velocity_tar){ //通信系から目標速度をもらう
		this->velocity_tar = velocity_tar;

	}
	float com_get_velocity(); //通信系に現在の速度を返す
	float get_velocity(); //エンコダ出力から現在の速度を計算して返す

	float com_get_current(); //通信系に現在の電流を返す
	float get_current(); //電流センサ出力から現在の電流を計算して返す

	void controller_velocity(); //速度制御を行う
	void controller_torque(); //トルク制御を行う

	PID pid_velocity;

	PID pid_torque;

	USER_CAN use_can;

};

extern STM_MotorSystem *_ms; //割り込みハンドラに同一のインスタンスを渡すためのポインタ(main program で宣言してあげて)

#endif /* INC_STM_MOTORSYSTEM_H_ */
