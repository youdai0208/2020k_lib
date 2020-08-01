/*
 * OmuniWheel.cpp
 *
 *  Created on: 2019/06/26
 *      Author: youda
 */

#include "MyLibInc/OmuniWheel.hpp"

OmuniWheel::OmuniWheel(){
//	Init(can_handle, distance_to_wheel, radius_of_wheel);
	//	motorDriver_ = MotorDriver(can_handle);

//	Init();

	/*motorDriver_.PIDinit((0 | 0x10), 0.0, 0.0, 0.0, 20, 4096);
	motorDriver_.PIDinit((1 | 0x10), 0.0, 0.0, 0.0, 20, 4096);
	motorDriver_.PIDinit((2 | 0x10), 0.0, 0.0, 0.0, 20, 4096);
	motorDriver_.PIDinit((3 | 0x10), 0.0, 0.0, 0.0, 20, 4096);*/
}

OmuniWheel::~OmuniWheel(){

}

void OmuniWheel::Init(const CAN_HandleTypeDef &can_handle){
	motorDriver_.Init(can_handle);
	/*motorDriver_.PIDInit((0 | 0x10), 0.015, 0.00025, 0.150, 1000, 4096);
	motorDriver_.PIDInit((1 | 0x10), 0.015, 0.00020, 0.125, 1000, 4096);
	motorDriver_.PIDInit((2 | 0x10), 0.015, 0.00025, 0.150, 1000, 4096);
	motorDriver_.PIDInit((3 | 0x10), 0.015, 0.00025, 0.150, 1000, 4096);*/
	motorDriver_.PIDInit((0 | 0x10), 0.020, 0.00050, 0.060, 1000, 4096);
	motorDriver_.PIDInit((1 | 0x10), 0.010, 0.00050, 0.050, 1000, 4096);
	motorDriver_.PIDInit((2 | 0x10), 0.020, 0.00050, 0.050, 1000, 4096);
	motorDriver_.PIDInit((3 | 0x10), 0.020, 0.00050, 0.050, 1000, 4096);
}

std::array<double, 4> OmuniWheel::calEachWheelSpeed(const Vector next_vector){
	static Vector last_vector = {.x = 0.0, .y = 0.0, .theta = 0.0};
	std::array<double, 4> result_array;
	Vector cal_speed_vector;
	Vector after_turn_vector;
	Eigen::MatrixXd a(4, 3);
	Eigen::MatrixXd b(3, 1);
	Eigen::MatrixXd result(4,1);

	/*after_turn_vector.x = (next_vector.x * cos(-(M_PI / 4.0)))
			- (next_vector.y * sin(-(M_PI / 4.0)));
	after_turn_vector.y = (next_vector.y * cos(-(M_PI / 4.0)))
			- (next_vector.x * sin(-(M_PI / 4.0)));
	after_turn_vector.theta = next_vector.theta - 45.0;*/

	after_turn_vector = turnCoordinate(vectorLimitCalculation(next_vector), -45.0);

	cal_speed_vector.x = (last_vector.x + (after_turn_vector.x - (last_vector.x /* 10.0*/))) / kInterruptRateTime;
	cal_speed_vector.y = (last_vector.y + (after_turn_vector.y - (last_vector.y /* 10.0*/))) / kInterruptRateTime;
	cal_speed_vector.theta = (((last_vector.theta * M_PI) / 180.0) + ((after_turn_vector.theta * M_PI) / 180.0) - ((last_vector.theta * M_PI) / 180.0)) / kInterruptRateTime;
//	cal_speed_vector.theta = (last_vector.theta + (after_turn_vector.theta - last_vector.theta)) / kInterruptRateTime;

	cal_speed_vector = calAccelToSpeed(cal_speed_vector);

	/*if(std::hypot((cal_speed_vector.x / 1000.0), (cal_speed_vector.y / 1000.0)) > kTargetSpeed){
		cal_speed_vector.x = kTargetSpeed * cos(atan2((cal_speed_vector.y / 1000.0), (cal_speed_vector.x / 1000.0)));
		cal_speed_vector.y = kTargetSpeed * sin(atan2((cal_speed_vector.y / 1000.0), (cal_speed_vector.x / 1000.0)));
	}else if(std::hypot((cal_speed_vector.x / 1000.0), (cal_speed_vector.y / 1000.0)) < (kTargetSpeed * (-1))){
		cal_speed_vector.x = (kTargetSpeed * (-1)) * cos(atan2((cal_speed_vector.y / 1000.0), (cal_speed_vector.x / 1000.0)));
		cal_speed_vector.y = (kTargetSpeed * (-1)) * sin(atan2((cal_speed_vector.y / 1000.0), (cal_speed_vector.x / 1000.0)));
	}else {
		cal_speed_vector.x = cal_speed_vector.x / 1000.0;
		cal_speed_vector.y = cal_speed_vector.y / 1000.0;
	}*/

	last_vector = after_turn_vector;

	a << 0.0, (0.5 * kRadiusOfWheel), (kDistanceToWheel / kRadiusOfWheel),
		 ((-0.5) * kRadiusOfWheel), 0.0, (kDistanceToWheel / kRadiusOfWheel),
		 0.0, ((-0.5) * kRadiusOfWheel), (kDistanceToWheel / kRadiusOfWheel),
		 ((0.5) * kRadiusOfWheel), 0.0, (kDistanceToWheel / kRadiusOfWheel);

	b << (cal_speed_vector.x / 1000.0),
		 (cal_speed_vector.y / 1000.0),
		 cal_speed_vector.theta;
	/*b << cal_speed_vector.x,
		cal_speed_vector.y,
		cal_speed_vector.theta;*/

	result = a * b;

	for(uint8_t i = 0;i < kWheelsMDQuantity;i++){
		result_array.at(i) = ((result((i), 0) / (2 * M_PI)) * 60);
		if(result_array.at(i) > kMaxRPM){
			result_array.at(i) = kMaxRPM;
		}else if(result_array.at(i) < kMinRPM){
			result_array.at(i) = kMinRPM;
		}
	}

	/*result_array.at(0) = result(0,0) * (2 * M_PI) * 60;
	result_array.at(1) = result(1,0) * (2 * M_PI) * 60;
	result_array.at(2) = result(2,0) * (2 * M_PI) * 60;
	result_array.at(3) = result(3,0) * (2 * M_PI) * 60;*/

	return result_array;
}

void OmuniWheel::setEachWheelSpeed(const std::array<double, 4> each_wheel_speed){
	if(!is_Emergency_){
		for(uint8_t i = 0;i < kWheelsMDQuantity;i++){
			setWheelSpeed((i | 0x10), static_cast<float>(each_wheel_speed.at(i)));
		}
	}else {
		std::array<float, 4> stop_speed = {0.0f, 0.0f, 0.0f, 0.0f};
		for(uint8_t i = 0;i < kWheelsMDQuantity;i++){
			setWheelSpeed((i | 0x10), stop_speed.at(i));
		}
	}
}

void OmuniWheel::setEachWheelDuty(const std::array<double, 4> each_wheel_duty){
//	uint8_t send_duty_data[4] = {0};
//	int32_t escape = 0;
	int32_t duty_data = 0;
 	if(!is_Emergency_){
		for(uint8_t i = 0;i < kWheelsMDQuantity;i++){
			if(each_wheel_duty.at(i) > 0.90){
				duty_data = 90;
			}else if(each_wheel_duty.at(i) < -0.90){
				duty_data = -90;
			}else {
				duty_data = static_cast<int32_t>(each_wheel_duty.at(i) * 100.0);
			}
//			std::memcpy(&escape, &duty_data, 4);
//			std::memcpy(send_duty_data, &duty_data, 4);
			/*send_duty_data[1] = static_cast<uint8_t>(duty_data >> 24);
			send_duty_data[2] = static_cast<uint8_t>(duty_data >> 16);
			send_duty_data[3] = static_cast<uint8_t>(duty_data >> 8);
			send_duty_data[4] = static_cast<uint8_t>(duty_data);*/
			motorDriver_.setDuty((i | 0x10), duty_data);
//			motorDriver_.update((i | 0x10), motorDriver_.drive_command::duty, send_duty_data);
		}
	}else {
		for(uint8_t i = 0;i < kWheelsMDQuantity;i++){
			motorDriver_.setDuty((i | 0x10), duty_data);
//			motorDriver_.update((i | 0x10), motorDriver_.drive_command::duty, send_duty_data);
		}
	}
}

void OmuniWheel::setEachWheelStop(){
	std::array<double, 4> stop_speed = {0.0, 0.0, 0.0, 0.0};
	setEachWheelSpeed(stop_speed);
//	setEachWheelDuty(stop_speed);
}

void OmuniWheel::setWheelSpeed(const uint8_t address, const float wheel_speed){
//	uint8_t send_data[4] = {0};
//	int32_t escape = 0;
	if(!is_Emergency_){
//		while(!motorDriver_.setTargetRPM(address, static_cast<int32_t>(wheel_speed)));
		motorDriver_.setTargetRPM(address, static_cast<int32_t>(wheel_speed));
//		motorDriver_.setDuty()
		/*std::memcpy(&escape, &wheel_speed, 4);
		std::memcpy(send_data, &wheel_speed, 4);
		send_data[0] = static_cast<uint8_t>(escape >> 24);
		send_data[1] = static_cast<uint8_t>(escape >> 16);
		send_data[2] = static_cast<uint8_t>(escape >> 8);
		send_data[3] = static_cast<uint8_t>(escape);*/
	}

	/*send_data[0] = (uint8_t)(wheel_speed >> 24);
	send_data[1] = (uint8_t)(wheel_speed >> 16);
	send_data[2] = (uint8_t)(wheel_speed >> 8);
	send_data[3] = (uint8_t)(wheel_speed);*/

//	motorDriver_.update(address, motorDriver_.drive_command::PID, send_data);
//	motorDriver_.setTargetRPM(address, static_cast<int32_t>(wheel_speed));
}

Vector OmuniWheel::calAccelToSpeed(const Vector now_target_speed){
	static Vector last_speed = {.x = 0.0, .y = 0.0, .theta = 0.0};
	static Vector cal_result = {.x = 0.0, .y = 0.0, .theta = 0.0};

	if((now_target_speed.x - last_speed.x) > kMaxAccel){
		cal_result.x = last_speed.x + (kMaxAccel * kInterruptRateTime);
	}else if((now_target_speed.x - last_speed.x) < kMinAccel){
		cal_result.x = last_speed.x + (kMinAccel * kInterruptRateTime);
	}else {
		cal_result.x = last_speed.x + ((now_target_speed.x - last_speed.x)/* * kInterruptRateTime*/);
	}

	if((now_target_speed.y - last_speed.y) > kMaxAccel){
		cal_result.y = last_speed.y + (kMaxAccel * kInterruptRateTime);
	}else if((now_target_speed.y - last_speed.y) < kMinAccel){
		cal_result.y = last_speed.y + (kMinAccel * kInterruptRateTime);
	}else {
		cal_result.y = last_speed.y + ((now_target_speed.y - last_speed.y));
	}

	if(now_target_speed.theta > kMaxAngle){
		cal_result.theta = kMaxAngle;
	}else if(now_target_speed.theta < kMinAngle) {
		cal_result.theta = kMinAngle;
	}else {
		cal_result.theta = now_target_speed.theta;
	}

	last_speed.x = cal_result.x;
	last_speed.y = cal_result.y;
	last_speed.theta = cal_result.theta;

	return cal_result;
}

Vector OmuniWheel::vectorLimitCalculation(const Vector before_vector){
	Vector result_vector;
	if(before_vector.x > kMaxVector){
		result_vector.x = kMaxVector;
	}else if(before_vector.x < kMinVector){
		result_vector.x = kMinVector;
	}else {
		result_vector.x = before_vector.x;
	}

	if(before_vector.y > kMaxVector){
		result_vector.y = kMaxVector;
	}else if(before_vector.y < kMinVector){
		result_vector.y = kMinVector;
	}else {
		result_vector.y = before_vector.y;
	}

	if(before_vector.theta > kMaxAngle){
		result_vector.theta = kMaxAngle;
	}else if(before_vector.theta < kMinAngle){
		result_vector.theta = kMinAngle;
	}else {
		result_vector.theta = before_vector.theta;
	}

	if(((std::abs(result_vector.x) + std::abs(result_vector.y)) >= 75.0) && (std::abs(result_vector.theta) >= kMaxAngle)){
		result_vector.x = 45.0;
		result_vector.y = 45.0;
		result_vector.theta = 5.0;
	}

	return result_vector;
}
