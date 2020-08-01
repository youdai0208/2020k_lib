/*
 * SelfPosition.cpp
 *
 *  Created on: 2019/06/01
 *      Author: youda
 */

#include "MyLibInc/SelfPosition.hpp"

SelfPosition::SelfPosition(){
	Init();
}

SelfPosition::~SelfPosition(){

}

void SelfPosition::Init(){
	machine_theta_ = 0.0;
	last_machine_theta_ = 0.0;
	calBeforeEachAxisEncoder_ = {0, 0};
	calAfterEachAxisEncoder_ = {0.0, 0.0};
	amountOfMovement_ = {0.0, 0.0};
	globalCoordinates_ = {0.0, 0.0};
//	imuData_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

EachAxisEncoder SelfPosition::calculationEachEncoderVector(const RawEachAxisEncoder &now_encoder_value, const double machine_theta){
	std::array<int16_t, 2> each_axis_encoder_value = {0, 0};
	EachAxisEncoder result_each_axis_encoder_value = {.x = 0.0, .y = 0.0};
//	static double last_machine_theta = 0.0;
	/*machine_theta_ += now_imu_data.gyro_z;
	set_machine_theta_(get_machine_theta_() + now_imu_data.gyro_z);*/
	each_axis_encoder_value.at(0) = static_cast<int16_t>(now_encoder_value.x);
	each_axis_encoder_value.at(1) = static_cast<int16_t>(now_encoder_value.y);

	set_machine_theta_(machine_theta);

	if((each_axis_encoder_value.at(0) != 0) || (each_axis_encoder_value.at(1) != 0)){
		/*set_x_axis_cal_after_encoder((static_cast<double>(each_axis_encoder_value.at(0)) * kEncoderDistancePerPulse)
			- (kxEncoderRelativeDistance * get_movement_machine_theta()));
		set_y_axis_cal_after_encoder((static_cast<double>(each_axis_encoder_value.at(1)) * kEncoderDistancePerPulse)
			- (kyEncoderRelativeDistance * get_movement_machine_theta()));*/

		calAfterEachAxisEncoder_.x = (static_cast<double>(each_axis_encoder_value.at(0)) * kEncoderDistancePerPulse)
					- (kxEncoderRelativeDistance * ((get_movement_machine_theta() * M_PI) / 180.0));
		calAfterEachAxisEncoder_.y = (static_cast<double>(each_axis_encoder_value.at(1)) * kEncoderDistancePerPulse)
					- (kyEncoderRelativeDistance * ((get_movement_machine_theta() * M_PI) / 180.0));

//		set_last_machine_theta(get_machine_theta_());

//		return get_after_each_axis_encoder();
		return calAfterEachAxisEncoder_;
	}else {
		return result_each_axis_encoder_value;
	}


//	set_after_each_axis_encoder(cal_each_encoder_vector_result);

}

AmountOfMovement SelfPosition::calculationAmountOfMovement(const EachAxisEncoder &finish_calculation_encoder_vector){

	/*set_x_axis_amount_of_movement(((finish_calculation_encoder_vector.x * sin(((get_last_machine_theta() + kyEncoderRelativeAngle) * M_PI) / 180.0))
			- (finish_calculation_encoder_vector.y * sin(((get_last_machine_theta() + kxEncoderRelativeAngle) * M_PI) / 180.0)))
					/ (-sin(((kxEncoderRelativeAngle - kyEncoderRelativeAngle) * M_PI) / 180.0)));
	set_y_axis_amount_of_movement(((finish_calculation_encoder_vector.x * cos(((get_last_machine_theta() + kyEncoderRelativeAngle) * M_PI) / 180.0))
			- (finish_calculation_encoder_vector.y * cos(((get_last_machine_theta() + kxEncoderRelativeAngle) * M_PI) / 180.0)))
					/ (sin(((kxEncoderRelativeAngle - kyEncoderRelativeAngle) * M_PI) / 180.0)));*/

	amountOfMovement_.x = ((finish_calculation_encoder_vector.x * sin(((get_last_machine_theta() + kyEncoderRelativeAngle) * M_PI) / 180.0))
			- (finish_calculation_encoder_vector.y * sin(((get_last_machine_theta() + kxEncoderRelativeAngle) * M_PI) / 180.0)))
					/ (-sin(((kxEncoderRelativeAngle - kyEncoderRelativeAngle) * M_PI) / 180.0));
	amountOfMovement_.y = ((finish_calculation_encoder_vector.x * cos(((get_last_machine_theta() + kyEncoderRelativeAngle) * M_PI) / 180.0))
			- (finish_calculation_encoder_vector.y * cos(((get_last_machine_theta() + kxEncoderRelativeAngle) * M_PI) / 180.0)))
					/ (sin(((kxEncoderRelativeAngle - kyEncoderRelativeAngle) * M_PI) / 180.0));

//	set_amount_of_movement(cal_amount_of_movement_result);
//	return get_amount_of_movement();
	return amountOfMovement_;
}

Coordinate SelfPosition::amountOfMovementToGlobalCoordinate(const AmountOfMovement &finish_calculation_amount_of_movement){
//	GlobalCoordinates cal_global_coordinates_result;

	set_x_axis_global_coordinate(get_x_axis_global_coordinate() + finish_calculation_amount_of_movement.x);
	set_y_axis_global_coordinate(get_y_axis_global_coordinate() + finish_calculation_amount_of_movement.y);

//	set_global_coordinates(cal_global_coordinates_result);
	return get_global_coordinate();
}

Coordinate SelfPosition::calculationGlobalCoordinates(const RawEachAxisEncoder &now_encoder_value, const double machine_theta) {
	set_after_each_axis_encoder(calculationEachEncoderVector(now_encoder_value, machine_theta));
	set_amount_of_movement(calculationAmountOfMovement(get_after_each_axis_encoder()));
	set_global_coordinates(amountOfMovementToGlobalCoordinate(get_amount_of_movement()));
//	set_last_machine_theta(get_machine_theta_());

	return get_global_coordinate();
}

