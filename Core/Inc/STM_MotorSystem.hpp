/*
 * STM_MotorSystem.h
 *
 *  Created on: 2021/09/11
 *      Author: 0_hayate
 */

#include <CAN.hpp>
#include <PID.hpp>
#include <stdio.h>
#include "main.h"

#ifndef INC_STM_MOTORSYSTEM_H_
#define INC_STM_MOTORSYSTEM_H_


class STM_MotorSystem
{
private:
	float velocity_ref; //エンコダからフィードバックされる速度
	float velocity_tar; //外部MCUから入力される目標速度

	float current_ref; //電流センサからフィードバックされる電流値

	float volt; //電源電圧

public:
	STM_MotorSystem(); //コンストラクタ

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

};

extern STM_MotorSystem *_ms;

#endif /* INC_STM_MOTORSYSTEM_H_ */
