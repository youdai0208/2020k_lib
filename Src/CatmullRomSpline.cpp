/*
 * CatmullRomSpline.cpp
 *
 *  Created on: 2019/05/31
 *      Author: youda
 */

#include "MyLibInc/CatmullRomSpline.hpp"

CatmullRomSpline::CatmullRomSpline(
	/*const std::vector<std::array<float, 2>> &point_data*/
	){
		/*const unsigned int n = static_cast<unsigned int>(point_data.size());
		assert(n >= 2);
		pointData_.resize(n + 2);
		copy(point_data.begin(), point_data.end(), pointData_.begin() + 1);
		pointData_[0] = point_data[0];
		pointData_[pointData_.size() - 1] = point_data[point_data.size() - 1];*/
}

CatmullRomSpline::~CatmullRomSpline(){

}

void CatmullRomSpline::Init(const std::vector<std::array<float, 2>> &point_data){
	const unsigned int n = static_cast<unsigned int>(point_data.size());
	assert(n >= 2);
	point_list_ = point_data;
	pointData_.resize(n + 2);
	std::copy(point_data.begin(), point_data.end(), pointData_.begin() + 1);
	pointData_[0] = point_data[0];
	pointData_[pointData_.size() - 1] = point_data[point_data.size() - 1];
	sequence_number_ = 0;
	in_sequence_resolution_number_ = 0;
	targetPointCoordinate_.x = 0.0;
	targetPointCoordinate_.y = 0.0;
	is_before_finish_ = false;
	is_last_sequence_ = false;
	is_sequence_finish_ = false;
}

Coordinate CatmullRomSpline::generateTargetPoint(const unsigned int idx, const float tim){
	assert(tim >= 0.0f && tim <= 1.0f);
	assert(idx < pointData_.size());

	Coordinate position;
//	array<float, 2> pos;
	std::array<float, 2> &p1 = pointData_[idx];
	std::array<float, 2> &p2 = pointData_[idx + 1];
	std::array<float, 2> &p3 = pointData_[idx + 2];
	std::array<float, 2> &p4 = pointData_[idx + 3];
	std::array<float, 2> v0 = { (p3.at(0) - p1.at(0)) * kPower,(p3.at(1) - p1.at(1)) * kPower };   // (p3 - p1) * power (Usually power is 0.5f)
	std::array<float, 2> v1 = { (p4.at(0) - p2.at(0)) * kPower,(p4.at(1) - p2.at(1)) * kPower };   // (p4 - p2) * power (Usually power is 0.5f)

	/*pos.at(0)*/position.x = calTargetPoint(p2.at(0), p3.at(0), v0.at(0), v1.at(0), tim);
	/*pos.at(1)*/position.y = calTargetPoint(p2.at(1), p3.at(1), v0.at(1), v1.at(1), tim);

	return position;
}

Coordinate CatmullRomSpline::getTargetPoint(const Vector &now_vector){
//	static uint8_t i = 0;
//	static uint8_t j = 0;
	static Coordinate last_target_coordinate;
	static double last_vector_hypotenuse = 0.0;
	static double last_last_vector_hypotenuse = 0.0;
	static Coordinate target_point_diffrence;
//	array<float, 2> v;
	double now_vector_hypotenuse = 0.0;
//	Vector next_vector;
	Coordinate diffrence_coordinate;

	/*if(get_is_sequence_finish_() == true){

		return finished_vector;
	}*/

	now_vector_hypotenuse = std::hypot(now_vector.x, now_vector.y);

	if (get_is_before_finish_() == true){
//		if((std::abs(now_vector.x) < kFinishVector) && (std::abs(now_vector.y) < kFinishVector)){
		if(std::abs(now_vector_hypotenuse) < kFinishVector){
			is_sequence_finish_ = true;
		}
		return targetPointCoordinate_;
	}

//	next_vector = now_vector;

//	while((std::abs(next_vector.x) < kChengeVector) && (std::abs(next_vector.y) < kChengeVector)){
	while(std::abs(now_vector_hypotenuse) < kChengeVector){
//	if(std::abs(now_vector_hypotenuse) < kChengeVector){
		in_sequence_resolution_number_++;
		if (in_sequence_resolution_number_ > kMathResolution) {
			in_sequence_resolution_number_ = 0;
			sequence_number_++;
		}

		if(pointData_.size() < 5){
			set_is_last_sequence_(true);
			if(in_sequence_resolution_number_ == static_cast<uint16_t>(kResolution - 1)){
				set_is_before_finish_(true);
				targetPointCoordinate_ = generateTargetPoint(
						sequence_number_,
						1.0f);
				break;
			}
		}

		if(sequence_number_ > (pointData_.size() - 5)){
			set_is_last_sequence_(true);
			if(in_sequence_resolution_number_ > static_cast<uint16_t>(kResolution - 1)){
				set_is_before_finish_(true);
				targetPointCoordinate_ = generateTargetPoint(
						sequence_number_,
						1.0f);
				break;
			}
		}

		targetPointCoordinate_ = generateTargetPoint(sequence_number_, (in_sequence_resolution_number_ / kResolution));

		/*if (sequence_number_ == 0) {
			if(pointData_.size() < 5){
				set_is_last_sequence_(true);
				if(in_sequence_resolution_number_ > (kMathResolution - static_cast<uint16_t>(kChengeVector))){
					targetPointCoordinate_ = generateTargetPoint(
							sequence_number_,
							1.0f);
					set_is_before_finish_(true);
				}else {
//					targetPointCoordinate_ = generateTargetPoint(
//							sequence_number_,
//							static_cast<float>((in_sequence_resolution_number_ + 5.0f) / kResolution));
					targetPointCoordinate_ = generateTargetPoint(
							sequence_number_,
							(static_cast<float>(in_sequence_resolution_number_) + kChengeVector) / kResolution);
				}
			}else {
//				if (in_sequence_resolution_number_ > (kMathResolution - 5)) {
				if(in_sequence_resolution_number_ > (kMathResolution - static_cast<uint16_t>(kChengeVector))){
//					targetPointCoordinate_ = generateTargetPoint(
//							sequence_number_ + 1,
//							static_cast<float>((in_sequence_resolution_number_ - (kResolution - 5.0f)) / kResolution));
					targetPointCoordinate_ = generateTargetPoint(
							sequence_number_ + 1,
							static_cast<float>((in_sequence_resolution_number_ - (kResolution - kChengeVector)) / kResolution));
					break;
				}else {
//					targetPointCoordinate_ = generateTargetPoint(
//							sequence_number_,
//							static_cast<float>((in_sequence_resolution_number_ + 5.0f) / kResolution));
					targetPointCoordinate_ = generateTargetPoint(
							sequence_number_,
							static_cast<float>((in_sequence_resolution_number_ + kChengeVector) / kResolution));
				}
			}

		}else {
			if (sequence_number_ >= (pointData_.size() - 2)) {
				set_is_last_sequence_(true);
				if (in_sequence_resolution_number_ > (kMathResolution - static_cast<uint16_t>(kChengeVector))) {
					targetPointCoordinate_ = generateTargetPoint(
							sequence_number_,
							1.0f);
					set_is_before_finish_(true);
				}else {
//					targetPointCoordinate_ = generateTargetPoint(
//							sequence_number_,
//							static_cast<float>((in_sequence_resolution_number_ + 5.0f) / kResolution));
					targetPointCoordinate_ = generateTargetPoint(
							sequence_number_,
							(static_cast<float>(in_sequence_resolution_number_) + kChengeVector) / kResolution);
				}
			}else {
//				if(in_sequence_resolution_number_ > (kMathResolution - 5)) {
				if(in_sequence_resolution_number_ > (kMathResolution - static_cast<uint16_t>(kChengeVector))) {
//					targetPointCoordinate_ = generateTargetPoint(
//							sequence_number_ + 1,
//							static_cast<float>((in_sequence_resolution_number_ - (kResolution - 5.0f)) / kResolution));
					targetPointCoordinate_ = generateTargetPoint(
							sequence_number_ + 1,
							static_cast<float>(in_sequence_resolution_number_ - (kResolution - kChengeVector)) / kResolution);
					break;
				}else {
//					targetPointCoordinate_ = generateTargetPoint(
//							sequence_number_,
//							static_cast<float>((in_sequence_resolution_number_ + 5.0f) / kResolution));
					targetPointCoordinate_ = generateTargetPoint(
							sequence_number_,
							static_cast<float>((in_sequence_resolution_number_ + kChengeVector) / kResolution));
				}
			}
		}*/

		diffrence_coordinate.x = targetPointCoordinate_.x - last_target_coordinate.x;
		diffrence_coordinate.y = targetPointCoordinate_.y - last_target_coordinate.y;
//		next_vector.x = diffrence_coordinate.x;
//		next_vector.y = diffrence_coordinate.y;
		now_vector_hypotenuse += std::hypot(diffrence_coordinate.x, diffrence_coordinate.y);

		/*target_point_diffrence.x = std::abs(point_list_.at(sequence_number_).at(0) - point_list_.at(sequence_number_ - 1).at(0));
		target_point_diffrence.y = std::abs(point_list_.at(sequence_number_).at(1) - point_list_.at(sequence_number_ - 1).at(1));*/

		/*if(std::abs(now_vector_hypotenuse - last_vector_hypotenuse) <
				(std::abs(last_vector_hypotenuse - last_last_vector_hypotenuse) -
						(std::hypot(point_list_.at(sequence_number_).at(0), point_list_.at(sequence_number_).at(1)) / kResolution))){*/
		/*if((target_point_diffrence.x == 0.0) && (target_point_diffrence.y != 0.0)){
			if(std::abs(now_vector_hypotenuse - last_vector_hypotenuse) < target_point_diffrence.y){
				break;
			}
		}else if((target_point_diffrence.x != 0.0) && (target_point_diffrence.y == 0.0)){
			if(std::abs(now_vector_hypotenuse - last_vector_hypotenuse) < target_point_diffrence.x){
				break;
			}
		}else {
			if(std::abs(now_vector_hypotenuse - last_vector_hypotenuse) <
				std::abs(std::hypot(target_point_diffrence.x, target_point_diffrence.y) / ((kResolution / 10) * 9))){
				break;
			}
		}

		last_last_vector_hypotenuse = last_vector_hypotenuse;*/
		last_vector_hypotenuse = now_vector_hypotenuse;
		last_target_coordinate = targetPointCoordinate_;

//		return targetPointCoordinate_;
	}/*else{*/
		return targetPointCoordinate_;
//	}
}

