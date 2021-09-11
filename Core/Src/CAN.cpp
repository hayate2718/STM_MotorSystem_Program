/*
 * CAN.cpp
 *
 *  Created on: 2021/09/11
 *      Author: 0_hayate
 */

#include <CAN.hpp>

typedef enum{
	SET_VELOCITY,
	SET_VELOCITY_P,
	SET_VELOCITY_I,
	SET_VELOCITY_D,

	SET_TORQUE,
	SET_TORQUE_P,
	SET_TORQUE_I,
	SET_TORQUE_D,

	GET_VELOCITY,
	GET_VELOCITY_P,
	GET_VELOCITY_I,
	GET_VELOCITY_D,

	GET_TORQUE_P,
	GET_TORQUE_I,
	GET_TORQUE_D,

	GET_CURRENT,

	SYSTEM_INIT,
	SYSTEM_START

}cmd;

USE_CAN::USE_CAN(){

	CAN_FilterTypeDef filter;
	_filter = &filter;

	CAN_TxHeaderTypeDef TxHeader;
	_TxHeader = &TxHeader;

}

void USE_CAN::use_tx_CAN(uint32_t cmd,float data){
	CAN_TxHeaderTypeDef *TxHeader;
	TxHeader = _TxHeader;

	can_data tx;

	uint32_t mailbox;

	tx.low_data = data;
	TxHeader->StdId = cmd;

	HAL_CAN_AddTxMessage(_use_hcan,TxHeader, tx.low_data_raw,&mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	CAN_RxHeaderTypeDef RxHeader;
	can_data rx;

	STM_MotorSystem *ms = _ms;

	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx.low_data_raw) == HAL_OK){
			switch(RxHeader.StdId){
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



