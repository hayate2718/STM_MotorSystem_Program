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

public :
	USER_CAN(); //CAN通信のフィルタとかの設定

	void use_tx_CAN(uint32_t cmd,float data); //CAN通信の送信関数

	uint8_t set_id_CAN();

	CAN_HandleTypeDef * _use_hcan;

};

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan); //受信割り込みコールバック

#endif /* INC_CAN_HPP_ */
