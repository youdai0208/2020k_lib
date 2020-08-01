/*
 * SelfPosition.hpp
 *
 *  Created on: 2019/05/23
 *      Author: youda
 */

#ifndef SELFPOSITION_HPP_
#define SELFPOSITION_HPP_

#include <cmath>
#include <array>

#include "stm32f3xx_hal.h"

#include "MyLibInc/Type.hpp"

class SelfPosition {
	public:
	static constexpr double kxEncoderRelativeAngle = -180.0;  //(�ｿｽ�ｿｽ)
	static constexpr double kyEncoderRelativeAngle = 90.0;  //(�ｿｽ�ｿｽ)
	static constexpr double kxEncoderRelativeDistance = 271.740;  //(mm)
	static constexpr double kyEncoderRelativeDistance = 271.740;  //(mm)
	static constexpr double kEncoderCircumfrentialLength = 188.5;
	static constexpr double kEncoderPPR = 8192;
	static constexpr double kEncoderDistancePerPulse = kEncoderCircumfrentialLength / kEncoderPPR;

	 SelfPosition();
	 ~SelfPosition();
	 void Init();
	 inline void set_machine_theta_(const double now_theta){
		 last_machine_theta_ = machine_theta_;
		 machine_theta_ = now_theta;
	 }
	 inline double get_machine_theta_(void){
		 return machine_theta_;
	 }
	 inline Coordinate get_global_coordinate(void){
		 return globalCoordinates_;
	 }
	 inline void set_before_each_axis_encoder(RawEachAxisEncoder cal_before_each_axis_encoder){
	 	calBeforeEachAxisEncoder_ = cal_before_each_axis_encoder;
	 }
	 inline RawEachAxisEncoder get_before_each_axis_encoder(void){
	 	return calBeforeEachAxisEncoder_;
	 }
	 inline void set_after_each_axis_encoder(const EachAxisEncoder cal_after_each_axis_encoder){
	 	calAfterEachAxisEncoder_ = cal_after_each_axis_encoder;
	 }
	 inline EachAxisEncoder get_after_each_axis_encoder(void){
		return calAfterEachAxisEncoder_;
	 }
	 inline void set_amount_of_movement(const AmountOfMovement amount_of_movement){
	 	amountOfMovement_ = amount_of_movement;
	 }
	 inline AmountOfMovement get_amount_of_movement(void){
	 	return amountOfMovement_;
	 }
	 inline void set_global_coordinates(const Coordinate global_coordinates){
	 	globalCoordinates_ = global_coordinates;
	 }

	 EachAxisEncoder calculationEachEncoderVector(const RawEachAxisEncoder &now_encoder_value, const double machine_theta);
	 AmountOfMovement calculationAmountOfMovement(const EachAxisEncoder &finish_calculation_encoder_vector);
	 Coordinate amountOfMovementToGlobalCoordinate(const AmountOfMovement &finish_calculation_amount_of_movement);
	 Coordinate calculationGlobalCoordinates(const RawEachAxisEncoder &now_encoder_value, const double machine_theta);

	private:
	 static constexpr double M_PI = 3.14159265358979323846;
	 double machine_theta_;
	 double last_machine_theta_;
	 RawEachAxisEncoder calBeforeEachAxisEncoder_;
	 EachAxisEncoder calAfterEachAxisEncoder_;
	 AmountOfMovement amountOfMovement_;
	 Coordinate globalCoordinates_;
//	 IMUData imuData_;

	 inline void set_x_axis_cal_before_encoder(const uint32_t cal_before_x_axis_encoder){
		 calBeforeEachAxisEncoder_.x =cal_before_x_axis_encoder;
	 }
	 inline uint32_t get_x_axis_cal_before_encoder(void){
		 return calBeforeEachAxisEncoder_.x;
	 }
	 inline void set_y_axis_cal_before_encoder(const uint32_t cal_before_y_axis_encoder){
		 calBeforeEachAxisEncoder_.y = cal_before_y_axis_encoder;
	 }
	 inline uint32_t get_y_axis_cal_before_encoder(void){
		 return calBeforeEachAxisEncoder_.y;
	 }
	 inline void set_x_axis_cal_after_encoder(const double cal_after_x_axis_encoder){
		 calAfterEachAxisEncoder_.x = cal_after_x_axis_encoder;
	 }
	 inline double get_x_axis_cal_after_encoder(void){
		 return calAfterEachAxisEncoder_.x;
	 }
	 inline void set_y_axis_cal_after_encoder(const double cal_after_y_axis_encoder){
		 calAfterEachAxisEncoder_.y = cal_after_y_axis_encoder;
	 }
	 inline double get_y_axis_cal_after_encoder(void){
		 return calAfterEachAxisEncoder_.y;
	 }
	 inline void set_x_axis_amount_of_movement(const double x_axis_amount_of_movement){
		 amountOfMovement_.x = x_axis_amount_of_movement;
	 }
	 inline double get_x_axis_amount_of_movement(void){
		 return amountOfMovement_.x;
	 }
	 inline void set_y_axis_amount_of_movement(const double y_axis_amount_of_movement){
		 amountOfMovement_.y = y_axis_amount_of_movement;
	 }
	 inline double get_y_axis_amount_of_movement(void){
		 return amountOfMovement_.y;
	 }
	 inline void set_x_axis_global_coordinate(const double x_axis_global_coordinate){
		 globalCoordinates_.x = x_axis_global_coordinate;
	 }
	 inline double get_x_axis_global_coordinate(void){
		 return globalCoordinates_.x;
	 }
	 inline void set_y_axis_global_coordinate(const double y_axis_global_coordinate){
		 globalCoordinates_.y = y_axis_global_coordinate;
	 }
	 inline double get_y_axis_global_coordinate(void){
		 return globalCoordinates_.y;
	 }
	 inline double get_movement_machine_theta(void){
		 return (last_machine_theta_ - machine_theta_);
	 }
	 inline void set_last_machine_theta(double last_machine_theta){
		 last_machine_theta_ = last_machine_theta;
	 }
	 inline double get_last_machine_theta(void){
		 return last_machine_theta_;
	 }
};

#endif /* SELFPOSITION_HPP_ */
