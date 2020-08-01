/*
 * MoterDriver.hpp
 *
 *  Created on: 2019/06/29
 *      Author: youda
 */

#ifndef MYLIBINC_MOTORDRIVER_HPP_
#define MYLIBINC_MOTORDRIVER_HPP_

#include <array>
#include <cstring>
#include <cassert>

#include "stm32f3xx_hal.h"

#include "MyLibInc/Can.hpp"



class MotorDriver{
public:
	/*モータドライバのコマンドを定義*/
	enum class drive_command
	{
		kDuty			= 0x00,
		kPID			= 0x01,
		kSetParamP		= 0x02,
		kSetParamI		= 0x03,
		kSetParamD		= 0x04,
		kSetParamLIMIT	= 0x05,
		kSetParamPPM	= 0x06,
		kEmergency      = 0x80
	};

	MotorDriver();
	~MotorDriver();
	void Init(const CAN_HandleTypeDef &can_handle);
	void PIDInit(const uint8_t address, const float kp, const float ki, const float kd, const uint32_t max_rpm, const uint32_t kppm);
	bool setTargetRPM(const uint8_t address, const int32_t target_rpm);
	bool setDuty(const uint8_t address, const int32_t duty);
	bool setParameter(const uint8_t address, const drive_command mode, const float fparam_value);
	bool setParameter(const uint8_t address, const drive_command mode, const uint32_t uparam_value);
//	bool allUpdate(const uint8_t first_address, const uint8_t cmd, const uint8_t &send_data);
	bool Emergency(const uint8_t address);

private:
//	CAN_HandleTypeDef hcan_;
	Can can_;
	bool updateDataSend(const uint8_t address, const drive_command cmd, const uint8_t (&send_data)[4]);  //MD1枚のみのアップデート

};



#endif /* MYLIBINC_MOTORDRIVER_HPP_ */
