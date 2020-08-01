#include "MyLibInc/AutoRunning.hpp"

AutoRunning::AutoRunning(){

}

AutoRunning::~AutoRunning(){

}

void AutoRunning::Init(std::vector<std::array<float, 2>> &point_list, std::vector<double> &angle_list,
				CAN_HandleTypeDef &can_handle, I2C_HandleTypeDef &i2c_handle,
				TIM_HandleTypeDef &x_encoder, TIM_HandleTypeDef &y_encoder){
	if(sequence_ != Sequence::Init){

	}
	passingPointList_ = point_list;
	passingAngleList_ = angle_list;
	bno055_.AssignI2C(&i2c_handle);
	catmullRomSpline_.Init(passingPointList_);
	omuniWheel_.Init(can_handle);
	selfPosition_.Init();
	nowVector_.x = 0.0;
	nowVector_.y = 0.0;
	nowVector_.theta = 0.0;
	globalCoordinate_.x = 0.0;
	globalCoordinate_.y = 0.0;
	targetPoint_.x = 0.0;
	targetPoint_.y = 0.0;

	sequence_ = Sequence::Waiting;
}

void AutoRunning::UpDate(){
	BNO055::vector_t bno_vector;
	static double before_processing_last_yaw = 0.0;
	static double after_processing_last_yaw = 0.0;
	double after_processing_now_yaw = 0.0;
	double yaw_difference = 0.0;
	double before_processing_now_yaw = 0.0;
	if((sequence_ != Sequence::Emergency) || (sequence_ != Sequence::Init) || (sequence_ != Sequence::UnInit)){
		return;
	}

	bno_vector = bno055_.GetVectorEuler();
	if(bno_vector.x > 180.0){
		before_processing_now_yaw = (360.0 - bno_vector.x);
	}else {
		before_processing_now_yaw = bno_vector.x * (-1);
	}

	yaw_difference = before_processing_now_yaw - before_processing_last_yaw;
	if(yaw_difference > 180.0){
		yaw_difference = 360.0 - yaw_difference;
	}else if(yaw_difference < -180.0){
		yaw_difference = (360.0 - std::abs(yaw_difference)) * (-1);
	}
	after_processing_now_yaw = after_processing_last_yaw + yaw_difference;

	after_processing_last_yaw = after_processing_now_yaw;
	before_processing_last_yaw = before_processing_now_yaw;

	if((std::abs(bno_vector.y) < 360.0) && (std::abs(bno_vector.z) < 360.0)){
		pIMUData_.roll = bno_vector.z;
		pIMUData_.pitch = bno_vector.y;
		pIMUData_.yaw = after_processing_now_yaw;
	}

	if((std::abs(pIMUData_.roll) >= 15.0) || (std::abs(pIMUData_.pitch) >= 15.0)){
		sequence_ = Sequence::Emergency;
	}

	globalCoordinate_ = selfPosition_.calculationGlobalCoordinates(rawEachAxisEncoder_, pIMUData_.yaw);

	this->SequenceManagement();
}

void AutoRunning::SequenceManagement(){
	Vector turned_next_vector;
	switch(sequence_){
		case Sequence::UnInit:
			break;
		case Sequence::Init:
			break;
		case Sequence::Emergency:
			omuniWheel_.enableEmergency();
			break;
		case Sequence::Waiting:
			omuniWheel_.setEachWheelStop();
			break;
		case Sequence::DuringAutoRunning:
			targetPoint_ = catmullRomSpline_.getTargetPoint(nowVector_);
			if(catmullRomSpline_.get_is_sequence_finish_()){
				if((std::abs(nowVector_.x) < catmullRomSpline_.kFinishVector)
						&& (std::abs(nowVector_.y) < catmullRomSpline_.kFinishVector)
						&& (std::abs(nowVector_.theta) < 2.0)){
					sequence_ = Sequence::FiniahAutoRunning;
					break;
				}
			}

			nowVector_.x = (targetPoint_.x - globalCoordinate_.x);
			nowVector_.y = (targetPoint_.y - globalCoordinate_.y);
			nowVector_.theta = ((((passingAngleList_.at(catmullRomSpline_.get_sequence_number_() + 1) - passingAngleList_.at(catmullRomSpline_.get_sequence_number_()))
					/ static_cast<double>(catmullRomSpline_.kMathResolution)) * static_cast<double>(catmullRomSpline_.get_in_sequence_resolution_number_())) + passingAngleList_.at(catmullRomSpline_.get_sequence_number_())) - selfPosition_.get_machine_theta_();
			turned_next_vector = omuniWheel_.turnCoordinate(nowVector_, selfPosition_.get_machine_theta_() * (-1));

			turned_next_vector.x = (turned_next_vector.x / 45) * (-1);
			turned_next_vector.y = (turned_next_vector.y / 45) * (-1);
			turned_next_vector.theta = (turned_next_vector.theta / 20) * (-1);

			if(!(sequence_ == Sequence::Emergency)){
				result_data_ = omuniWheel_.calEachWheelSpeed(turned_next_vector);
			}
			break;
		case Sequence::FiniahAutoRunning:
			omuniWheel_.setEachWheelStop();
			break;
		default:
			omuniWheel_.enableEmergency();
			break;
	}
}

void calVector(){

}
