/*
 * STM_MotorSystem_init.cpp
 *
 *  Created on: Sep 11, 2021
 *      Author: 0_hayate
 */

#include <STM_MotorSystem.hpp>

STM_MotorSystem::STM_MotorSystem():
pid_velocity(0,0,0,0.0001),
pid_torque(0,0,0,0.0001),
velocity_ref(0),
velocity_tar(0),
current_ref(0),
volt(0)
{
	use_can._use_hcan = &hcan;


}

