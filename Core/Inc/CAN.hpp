/*
 * CAN.h
 *
 *  Created on: Sep 11, 2021
 *      Author: 0_hayate
 */


#ifndef INC_CAN_HPP_
#define INC_CAN_HPP_

#include "main.h"


#define _MAX_DLC 8

typedef enum{
	SET_VELOCITY = 0x710,
	SET_VELOCITY_P = 0x720,
	SET_VELOCITY_I = 0x730,
	SET_VELOCITY_D = 0x740,

	SET_TORQUE = 0x750,
	SET_TORQUE_P = 0x760,
	SET_TORQUE_I = 0x770,
	SET_TORQUE_D = 0xf80,

	SET_VOLTAGE =0x610, //電源電圧
	SET_PPR = 0x620, //エンコダ分解能
	SET_KT = 0x630, //モタトルク係数
	SET_CURRENT_LIMIT = 0x640, //電流制限（ストール電流以上に設定すると積分が死ぬ）
	SET_ADC_GAIN = 0x650, //電流センサのゲイン設定

	SET_COAST = 0x510,
	SET_RESET = 0x520, //追加予定

	GET_VELOCITY = 0x410,
	GET_VELOCITY_P = 0x420, //追加予定
	GET_VELOCITY_I = 0x430, //追加予定
	GET_VELOCITY_D = 0x440, //追加予定

	GET_TORQUE_P = 0x450, //追加予定
	GET_TORQUE_I = 0x460, //追加予定
	GET_TORQUE_D = 0x470, //追加予定

	GET_CURRENT = 0x480,

	GET_FF1_STATE = 0x310, //追加予定
	GET_FF2_STATE = 0x320, //追加予定

	SYSTEM_INIT = 0x010,
	SYSTEM_START = 0x020,
	MOTOR_SYSTEM_STOP = 0x030,

	ALERT_FF1 = 0x040, //追加予定
	ALERT_FF2 = 0x050, //追加予定
	ALERT_FF1_FF2 = 0x060, //追加予定

}cmd;

typedef union{
	struct {
		union{
			float low_data;
			uint8_t low_data_raw[4];
		};
		union{
			float high_data;
			uint8_t high_data_raw[4];
		};
	};
	uint8_t all_data[_MAX_DLC];
}can_data;

typedef union{
	struct{
		uint8_t bit0 : 1;
		uint8_t bit1 : 1;
		uint8_t bit2 : 1;
		uint8_t bit3 : 1;
	};
	uint8_t all_data:4;
}id_set;

class USER_CAN{
private :
	CAN_FilterTypeDef filter;
	CAN_TxHeaderTypeDef TxHeader;
	CAN_HandleTypeDef * _use_hcan;


public :
	USER_CAN(CAN_HandleTypeDef * _use_hcan); //CAN通信のフィルタとかの設定

	void use_tx_CAN(uint32_t cmd,float data); //CAN通信の送信関数
	void use_rx_CAN(CAN_HandleTypeDef * _hcan); //CAN通信の受信関数

	uint8_t get_id_CAN();

	void set_id_CAN(GPIO_TypeDef * GPIO_idbit0,
			GPIO_TypeDef * GPIO_idbit1,
			GPIO_TypeDef * GPIO_idbit2,
			GPIO_TypeDef * GPIO_idbit3,
			uint16_t GPIO_PIN_idbit0,
			uint16_t GPIO_PIN_idbit1,
			uint16_t GPIO_PIN_idbit2,
			uint16_t GPIO_PIN_idbit3
			);

	void filter_set();

	void set_dlc_CAN(uint32_t dlc);

	void set_rtr_CAN(uint32_t rtr);

	void set_ide_CAN(uint32_t ide);

	GPIO_TypeDef * GPIO_idbit0; //基板スイッチにつながるGPIOを指定する
	uint16_t GPIO_PIN_idbit0;

	GPIO_TypeDef * GPIO_idbit1;
	uint16_t GPIO_PIN_idbit1;

	GPIO_TypeDef * GPIO_idbit2;
	uint16_t GPIO_PIN_idbit2;

	GPIO_TypeDef * GPIO_idbit3;
	uint16_t GPIO_PIN_idbit3;

};

inline void USER_CAN::set_dlc_CAN(uint32_t dlc){
	TxHeader.DLC = dlc;
}

inline void USER_CAN::set_rtr_CAN(uint32_t rtr){
	TxHeader.RTR = rtr;
}

inline void USER_CAN::set_ide_CAN(uint32_t ide){
	TxHeader.IDE = ide;
}

#endif /* INC_CAN_HPP_ */
