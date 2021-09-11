/*
 * STM_MotorSystem_init.cpp
 *
 *  Created on: Sep 11, 2021
 *      Author: 0_hayate
 */

#include <STM_MotorSystem.hpp>

STM_MotorSystem::STM_MotorSystem():
pid_velocity(0,0,0,0.0001),
pid_torque(0,0,0,0.0001)
{

	USE_CAN use_can;
	use_can._use_hcan = &hcan;


}

