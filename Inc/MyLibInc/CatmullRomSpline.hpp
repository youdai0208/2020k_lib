/*
 * CatmullRomSpline.hpp
 *
 *  Created on: 2019/05/31
 *      Author: youda
 */

#ifndef MYLIBINC_CATMULLROMSPLINE_HPP_
#define MYLIBINC_CATMULLROMSPLINE_HPP_

#include <cassert>
#include <cmath>
#include <vector>
#include <array>

#include "MyLibInc/Type.hpp"


class CatmullRomSpline{
	public:
	 static constexpr double kFinishVector = 25.0;
	 static constexpr uint16_t kMathResolution = 5000;
	 static constexpr double kChengeVector = 450.0;
	 CatmullRomSpline(/*const std::vector<std::array<float, 2>> &point_data
			 = std::vector<std::array<float, 2>>()*/);
	 ~CatmullRomSpline();
	 inline bool get_is_before_finish_(){
		 return is_before_finish_;
	 }
	 inline void reset_is_before_finish_(){
		 is_before_finish_ = false;
	 }
	 inline bool get_is_sequence_finish_(){
		 return is_sequence_finish_;
	 }
	 inline bool get_is_last_sequence_(){
		 return is_last_sequence_;
	 }
	 inline uint16_t get_sequence_number_(){
		 return sequence_number_;
	 }
	 inline uint16_t get_in_sequence_resolution_number_(){
		 return in_sequence_resolution_number_;
	 }
	 void Init(const std::vector<std::array<float, 2>> &point_data = std::vector<std::array<float, 2>>());
	 Coordinate generateTargetPoint(const unsigned int idx, const float tim);
	 Coordinate getTargetPoint(const Vector &now_vector);

	private:
	 static constexpr float kPower = 0.5f;
//	 static constexpr uint16_t kMathResolution = 5000;
	 static constexpr float kResolution = static_cast<float>(kMathResolution);
	 bool is_before_finish_ = false;
	 bool is_sequence_finish_ = false;
	 bool is_last_sequence_ = false;
	 uint16_t in_sequence_resolution_number_ = 0;
	 uint16_t sequence_number_ = 0;
	 std::vector<std::array<float, 2>> pointData_;
	 std::vector<std::array<float, 2>> point_list_;
	 Coordinate targetPointCoordinate_;

	 inline void set_is_before_finish_(const bool state){
		 is_before_finish_ = state;
	 }
	 inline void set_is_last_sequence_(const bool state){
		 is_last_sequence_ = state;
	 }
	 inline float calTargetPoint(const float x0, const float x1, const float v0, const float v1, const float t){
		 return (2.0f * x0 - 2.0f * x1 + v0 + v1) * t * t * t
		 				+ (-3.0f * x0 + 3.0f * x1 - 2.0f * v0 - v1) * t * t
		 				+ v0 * t + x0;
	 }

};



#endif /* MYLIBINC_CATMULLROMSPLINE_HPP_ */
