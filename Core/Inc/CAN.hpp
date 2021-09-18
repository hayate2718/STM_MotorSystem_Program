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
	SET_VELOCITY = 0xf010,
	SET_VELOCITY_P = 0xf020,
	SET_VELOCITY_I = 0xf030,
	SET_VELOCITY_D = 0xf040,

	SET_TORQUE = 0xf110,
	SET_TORQUE_P = 0xf120,
	SET_TORQUE_I = 0xf130,
	SET_TORQUE_D = 0xf140,

	SET_VOLTAGE =0xf210, //電源電圧
	SET_PPR = 0xf220, //エンコダ分解能
	SET_KT = 0xf230, //モタトルク係数
	SET_CURRENT_LIMIT = 0xf240,

	SET_COAST = 0xf310,

	GET_VELOCITY = 0xe010,
	GET_VELOCITY_P = 0xe020,
	GET_VELOCITY_I = 0xe030,
	GET_VELOCITY_D = 0xe040,

	GET_TORQUE_P = 0xe110,
	GET_TORQUE_I = 0xe120,
	GET_TORQUE_D = 0xe130,

	GET_CURRENT = 0xe210,

	GET_FF1_STATE = 0xe310,
	GET_FF2_STATE = 0xe320,

	SYSTEM_INIT = 0x1010,
	SYSTEM_START = 0x1020,

	ALERT_FF1 = 0x0010,
	ALERT_FF2 = 0x0020,
	ALERT_FF1_FF2 = 0x0030,

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
	CAN_FilterTypeDef * _filter;
	CAN_TxHeaderTypeDef * _TxHeader;
	CAN_HandleTypeDef * _use_hcan;


public :
	USER_CAN(CAN_HandleTypeDef * _use_hcan); //CAN通信のフィルタとかの設定

	void use_tx_CAN(uint32_t cmd,float data); //CAN通信の送信関数

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


#endif /* INC_CAN_HPP_ */
