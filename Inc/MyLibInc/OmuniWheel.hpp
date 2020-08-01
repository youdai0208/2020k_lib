/*
 * OmuniWheel.hpp
 *
 *  Created on: 2019/06/26
 *      Author: youda
 */

#ifndef MYLIBINC_OMUNIWHEEL_HPP_
#define MYLIBINC_OMUNIWHEEL_HPP_

#define _USE_MATH_DEFINED

#include "MyLibInc/MotorDriver.hpp"
#include <array>
#include <cmath>
#include <cstring>

#include "MyLibInc/Type.hpp"
#include "stm32f3xx_hal.h"

#include "Eigen/Core"
#include "Eigen/Dense"


class OmuniWheel{
public:
	static constexpr uint8_t kMinMDNumber 		= 10;
	static constexpr uint8_t kWheelsMDQuantity 	= 4;
	static constexpr uint8_t kMaxMDNumber 		= kMinMDNumber + kWheelsMDQuantity;
	static constexpr double M_PI 				= 3.14159265358979323846;
	static constexpr double kDistanceToWheel 	= 304.9;		//mm
	static constexpr double kRadiusOfWheel 		= 64.0;			//mm
	static constexpr double kTargetSpeed		= 1.5;			//mm
	static constexpr double kMaxSpeed 			= 3.889;		//m/s
	static constexpr double kMinSpeed 			= -3.889;		//m/s
	static constexpr double kMaxAccel 			= 2500.0;		//mm
	static constexpr double kMinAccel 			= -2500.0;		//mm
	static constexpr double kMaxVector 			= 45.0;			//mm
	static constexpr double kMinVector 			= -45.0;		//mm
	static constexpr double kMaxRPM 			= 1000.0;
	static constexpr double kMinRPM 			= -1000.0;
	static constexpr double kMaxAngle 			= 5.0;
	static constexpr double kMinAngle 			= -5.0;
	static constexpr double kMaxRad 			= (M_PI / 9.0);
	static constexpr double kMinRad 			= -(M_PI / 9.0);

	OmuniWheel();
	~OmuniWheel();
	inline bool enableEmergency(){
		set_is_Emergency_(true);
		for(uint8_t i = 0;i < kWheelsMDQuantity;i++){
			if(motorDriver_.Emergency((i | 0x10)) == false){
				return false;
			}
		}
		return true;
	}
	inline bool disableEmergency(){
		set_is_Emergency_(false);
		float sendData[4] = {0.0f};
		for(uint8_t i = 0;i < kWheelsMDQuantity;i++){
			/*if(motorDriver_.update((i | 0x10), motorDriver_.drive_command::duty, sendData) != true){
				return false;
			}*/
			if(motorDriver_.setDuty((i | 0x10), sendData[i]) == false){
				return false;
			}
		}
		return true;
	}
	inline bool get_is_Emergency_(){
		return is_Emergency_;
	}
	inline Vector turnCoordinate(const Vector before_turn_coordinate, const double turn_angle, const bool is_theta_turn = false){
		Vector after_turn_vector = {.x = 0.0, .y = 0.0, .theta = 0.0};

		after_turn_vector.x = (before_turn_coordinate.x * cos((turn_angle * M_PI) / 180.0))
					- (before_turn_coordinate.y * sin((turn_angle * M_PI) / 180.0));
		after_turn_vector.y = ((before_turn_coordinate.y * cos((turn_angle * M_PI) / 180.0))
					+ (before_turn_coordinate.x * sin((turn_angle * M_PI) / 180.0)));

		if(is_theta_turn){
			if(before_turn_coordinate.theta != 0.0){
				after_turn_vector.theta = before_turn_coordinate.theta - turn_angle;
			}
		}else {
			if(before_turn_coordinate.theta != 0.0){
				after_turn_vector.theta = before_turn_coordinate.theta;
			}
		}

		return after_turn_vector;
	}
	void Init(const CAN_HandleTypeDef &can_handle);
	std::array<double, 4> calEachWheelSpeed(const Vector next_vector);
	void setEachWheelSpeed(const std::array<double, 4> each_wheel_speed);
	void setEachWheelDuty(const std::array<double, 4> each_wheel_duty);
	void setEachWheelStop();

private:
	static constexpr double kInterruptRateTime = 0.01;
	MotorDriver motorDriver_;
	bool is_Emergency_ = false;
//	bool is_stop_ = false;
	void setWheelSpeed(const uint8_t address, const float wheel_speed);
	Vector calAccelToSpeed(const Vector next_vector);
	Vector vectorLimitCalculation(const Vector before_vector);
	inline void set_is_Emergency_(const bool is_emergency){
		is_Emergency_ = is_emergency;
	}

};

#endif /* MYLIBINC_OMUNIWHEEL_HPP_ */
