/*
 * CAN.cpp
 *
 *  Created on: 2021/09/11
 *      Author: 0_hayate
 */

#include <CAN.hpp>
#include <STM_MotorSystem.hpp>

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

USER_CAN::USER_CAN(){

	CAN_FilterTypeDef filter;
	_filter = &filter;

	CAN_TxHeaderTypeDef TxHeader;
	_TxHeader = &TxHeader;

	filter.FilterActivation = 0; //filter enable
	filter.FilterBank = 0; //used filterbank 0
 	filter.FilterFIFOAssignment = 0; //rxdata to fifo0
	filter.FilterMode = 0; //filter mode is mask mode
	filter.FilterScale = 0; //filterscale is dual 16bits
	filter.FilterIdHigh = set_id_CAN();
	filter.FilterMaskIdHigh = 15;
	HAL_CAN_ConfigFilter(_use_hcan, _filter);

	TxHeader.DLC = 4; //データ長（4byte）
	TxHeader.IDE = 0; //標準識別子
	TxHeader.RTR = 0; //データフレーム (現状モータシステムからホストにデータ要求はしないと思うから)
	TxHeader.TransmitGlobalTime = DISABLE; //タイムスタンプ無効

}

void USER_CAN::use_tx_CAN(uint32_t cmd,float data){
	CAN_TxHeaderTypeDef *TxHeader;
	TxHeader = _TxHeader;

	can_data tx;

	uint32_t mailbox;

	tx.low_data = data;
	TxHeader->StdId = cmd+set_id_CAN();

	HAL_CAN_AddTxMessage(_use_hcan,TxHeader, tx.low_data_raw,&mailbox);
}

uint8_t USER_CAN::set_id_CAN(){
	id_set id;
	id.bit0 = HAL_GPIO_ReadPin(GPIO_idbit0,GPIO_PIN_idbit0);
	id.bit1 = HAL_GPIO_ReadPin(GPIO_idbit1,GPIO_PIN_idbit1);
	id.bit2 = HAL_GPIO_ReadPin(GPIO_idbit2,GPIO_PIN_idbit2);
	id.bit3 = HAL_GPIO_ReadPin(GPIO_idbit3,GPIO_PIN_idbit3);
	return id.all_data;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef RxHeader;
	can_data rx;
	uint32_t cmd;
	STM_MotorSystem *ms = _ms;

	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx.low_data_raw) == HAL_OK){
		cmd = RxHeader.StdId & 0xfff0;
		switch(cmd){
			case SET_VELOCITY:
				ms->set_velocity(rx.low_data);
				break;

			case SET_VELOCITY_P:
				ms->pid_velocity.PID_set_p(rx.low_data);
				break;

			case SET_VELOCITY_I:
				break;

			case SET_VELOCITY_D:
				break;

			case SET_TORQUE:
				break;

			case SET_TORQUE_P:
				break;

			case SET_TORQUE_I:
				break;

			case SET_TORQUE_D:
				break;

			case GET_VELOCITY:
				break;

			case GET_VELOCITY_P:
				break;

			case GET_VELOCITY_I:
				break;

			case GET_VELOCITY_D:
				break;

			case GET_TORQUE_P:
				break;

			case GET_TORQUE_I:
				break;

			case GET_TORQUE_D:
				break;

			case GET_CURRENT:
				break;

			case SYSTEM_INIT:
				break;

			case SYSTEM_START:
				break;
			}

		}
}



