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


	filter.FilterActivation = 1; //filter enable = 1
	filter.FilterBank = 0; //used filterbank 0
 	filter.FilterFIFOAssignment = 0; //rxdata to fifo0
	filter.FilterMode = 0; //filter mode is mask mode
	filter.FilterScale = 0; //filterscale is dual 16bits
	filter.FilterMaskIdHigh = 0xf << 5; //filter mask
	filter.SlaveStartFilterBank = 0;


	TxHeader.DLC = 4; //データ長（4byte）
	TxHeader.IDE = 0; //標準識別子
	TxHeader.RTR = 0; //データフレーム (現状モータシステムからホストにデータ要求はしないと思うから)
	TxHeader.TransmitGlobalTime = DISABLE; //タイムスタンプ無効

}

void USER_CAN::use_tx_CAN(uint32_t cmd,float data){

	can_data tx;

	uint32_t mailbox;

	tx.low_data = data;
	TxHeader.StdId = cmd+get_id_CAN();

	HAL_CAN_AddTxMessage(_use_hcan,&TxHeader, tx.low_data_raw,&mailbox);
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

void USER_CAN::filter_set(){

	filter.FilterIdHigh = this->get_id_CAN() << 5;

	filter.SlaveStartFilterBank = 0;

	HAL_CAN_ConfigFilter(_use_hcan, &filter);

}

void USER_CAN::use_rx_CAN(CAN_HandleTypeDef *_hcan){
	if(_hcan != _use_hcan){
		return;
	}

	STM_MotorSystem * ms = STM_MotorSystem::_ms;
	CAN_RxHeaderTypeDef RxHeader;
	can_data rx;

	if(HAL_CAN_GetRxMessage(_use_hcan, CAN_RX_FIFO0, &RxHeader, rx.low_data_raw) == HAL_OK){
		switch(RxHeader.StdId & 0xfff0){
			case SET_VELOCITY:
				ms->set_velocity(rx.low_data);
				break;

			case SET_VELOCITY_P:
				ms->set_velocity_p(rx.low_data);
				break;

			case SET_VELOCITY_I:
				ms->set_velocity_i(rx.low_data);
				break;

			case SET_VELOCITY_D:
				ms->set_velocity_d(rx.low_data);
				break;

			case SET_TORQUE:
				ms->set_torque(rx.low_data);
				break;

			case SET_TORQUE_P:
				ms->set_torque_p(rx.low_data);
				break;

			case SET_TORQUE_I:
				ms->set_torque_i(rx.low_data);
				break;

			case SET_TORQUE_D:
				ms->set_torque_d(rx.low_data);
				break;

			case SET_ANGLE:
				ms->set_angle(rx.low_data);
				break;

			case SET_ANGLE_P:
				ms->set_angle_p(rx.low_data);
				break;

			case SET_ANGLE_I:
				ms->set_angle_i(rx.low_data);
				break;

			case SET_ANGLE_D:
				ms->set_angle_d(rx.low_data);
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

			case SET_ADC_GAIN:
				ms->use_adc.ADC_set_gain(rx.low_data);
				break;

			case SET_COAST:
				ms->set_coast();
				break;

			case SET_RESET:
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

			case GET_SUM_ANGLE:
				ms->use_can.use_tx_CAN(GET_SUM_ANGLE,ms->com_get_sum_angle());
				break;

			case GET_ANGLE_P:
				break;

			case GET_ANGLE_I:
				break;

			case GET_ANGLE_D:
				break;

			case SYSTEM_INIT:
				ms->STM_MotorSystem_init();
				break;

			case SYSTEM_START:
				ms->STM_MotorSystem_start();
				break;

			case MOTOR_SYSTEM_STOP:
				ms->STM_MotorSystem_stop();
			}

		}
}

#ifndef debug

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){ //受信割り込みコールバック
 	STM_MotorSystem *ms = STM_MotorSystem::_ms;
	ms->use_can.use_rx_CAN(hcan);
}
#endif


