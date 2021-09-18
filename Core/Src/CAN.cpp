/*
 * CAN.cpp
 *
 *  Created on: 2021/09/11
 *      Author: 0_hayate
 */

#include <CAN.hpp>
#include <STM_MotorSystem.hpp>


USER_CAN::USER_CAN(CAN_HandleTypeDef * _use_hcan){

	this->_use_hcan = _use_hcan;

	CAN_FilterTypeDef filter;
	_filter = &filter;

	CAN_TxHeaderTypeDef TxHeader;
	_TxHeader = &TxHeader;

	filter.FilterActivation = 0; //filter enable
	filter.FilterBank = 0; //used filterbank 0
 	filter.FilterFIFOAssignment = 0; //rxdata to fifo0
	filter.FilterMode = 0; //filter mode is mask mode
	filter.FilterScale = 0; //filterscale is dual 16bits
	filter.FilterIdHigh = get_id_CAN();
	filter.FilterMaskIdHigh = 15;
	HAL_CAN_ConfigFilter(this->_use_hcan, _filter);

	TxHeader.DLC = 4; //データ長（4byte）
	TxHeader.IDE = 0; //標準識別子
	TxHeader.RTR = 0; //データフレーム (現状モータシステムからホストにデータ要求はしないと思うから)
	TxHeader.TransmitGlobalTime = DISABLE; //タイムスタンプ無効

	HAL_CAN_Start(this->_use_hcan);
	HAL_CAN_ActivateNotification(this->_use_hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

}

void USER_CAN::use_tx_CAN(uint32_t cmd,float data){
	can_data tx;

	uint32_t mailbox;

	tx.low_data = data;
	_TxHeader->StdId = cmd+get_id_CAN();

	HAL_CAN_AddTxMessage(_use_hcan,_TxHeader, tx.low_data_raw,&mailbox);
}

uint8_t USER_CAN::get_id_CAN(){
	id_set id;
	id.bit0 = HAL_GPIO_ReadPin(GPIO_idbit0,GPIO_PIN_idbit0);
	id.bit1 = HAL_GPIO_ReadPin(GPIO_idbit1,GPIO_PIN_idbit1);
	id.bit2 = HAL_GPIO_ReadPin(GPIO_idbit2,GPIO_PIN_idbit2);
	id.bit3 = HAL_GPIO_ReadPin(GPIO_idbit3,GPIO_PIN_idbit3);
	return id.all_data;
}

void USER_CAN::set_id_CAN(GPIO_TypeDef * GPIO_idbit0,
			GPIO_TypeDef * GPIO_idbit1,
			GPIO_TypeDef * GPIO_idbit2,
			GPIO_TypeDef * GPIO_idbit3,
			uint16_t GPIO_PIN_idbit0,
			uint16_t GPIO_PIN_idbit1,
			uint16_t GPIO_PIN_idbit2,
			uint16_t GPIO_PIN_idbit3
			){
	this->GPIO_idbit0 = GPIO_idbit0;
	this->GPIO_idbit1 = GPIO_idbit1;
	this->GPIO_idbit2 = GPIO_idbit2;
	this->GPIO_idbit3 = GPIO_idbit3;
	this->GPIO_PIN_idbit0 = GPIO_PIN_idbit0;
	this->GPIO_PIN_idbit1 = GPIO_PIN_idbit1;
	this->GPIO_PIN_idbit2 = GPIO_PIN_idbit2;
	this->GPIO_PIN_idbit3 = GPIO_PIN_idbit3;
}

void USER_CAN::set_dlc_CAN(uint32_t dlc){
	_TxHeader->DLC = dlc;
}

void USER_CAN::set_rtr_CAN(uint32_t rtr){
	_TxHeader->RTR = rtr;
}

void USER_CAN::set_ide_CAN(uint32_t ide){
	_TxHeader->IDE = ide;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){ //受信割り込みコールバック
	CAN_RxHeaderTypeDef RxHeader;
	can_data rx;
	uint32_t cmd;
	STM_MotorSystem *ms = STM_MotorSystem::_ms;

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
				ms->pid_velocity.PID_set_i(rx.low_data);
				break;

			case SET_VELOCITY_D:
				ms->pid_velocity.PID_set_d(rx.low_data);
				break;

			case SET_TORQUE:
				ms->set_torque(rx.low_data);
				break;

			case SET_TORQUE_P:
				ms->pid_torque.PID_set_p(rx.low_data);
				break;

			case SET_TORQUE_I:
				ms->pid_torque.PID_set_i(rx.low_data);
				break;

			case SET_TORQUE_D:
				ms->pid_torque.PID_set_d(rx.low_data);
				break;

			case SET_VOLTAGE:
				ms->set_volt(rx.low_data);
				break;

			case SET_PPR:
				ms->set_ppr(rx.low_data);
				break;

			case SET_KT:
				ms->set_kt(rx.low_data);
				break;

			case SET_CURRENT_LIMIT:
				ms->set_current_limit(rx.low_data);
				break;

			case GET_VELOCITY:
				ms->use_can.use_tx_CAN(GET_VELOCITY,ms->com_get_velocity());
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
				ms->use_can.use_tx_CAN(GET_CURRENT,ms->com_get_current());
				break;

			case SYSTEM_INIT:
				ms->STM_MotorSystem_init();
				break;

			case SYSTEM_START:
				ms->STM_MotorSystem_start();
				break;
			}

		}
}



