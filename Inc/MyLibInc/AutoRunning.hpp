/*
 * AutoRunning.hpp
 *
 *  Created on: 2019/09/07
 *      Author: youda
 */

#ifndef MYLIBINC_AUTORUNNING_HPP_
#define MYLIBINC_AUTORUNNING_HPP_

#include "MyLibInc/BNO055.hpp"
#include "MyLibInc/CatmullRomSpline.hpp"
#include "MyLibInc/OmuniWheel.hpp"
#include "MyLibInc/SelfPosition.hpp"
#include "MyLibInc/Type.hpp"

class AutoRunning {
	public:
		AutoRunning();
		~AutoRunning();
		enum class Sequence {
			UnInit,
			Init,
			Emergency,
			Waiting,
			DuringAutoRunning,
			//finishのスペル
			FiniahAutoRunning,
		};
		std::array<double, 4> result_data_ = {0.0};
		void Init(std::vector<std::array<float, 2>> &point_list, std::vector<double> &angle_list,
				CAN_HandleTypeDef &can_handle, I2C_HandleTypeDef &i2c_handle,
				TIM_HandleTypeDef &x_encoder, TIM_HandleTypeDef &y_encoder);
		void UpDate();

		inline void EnableEmergency(){
			escapeSequence_ = sequence_;
			sequence_ = Sequence::Emergency;
			omuniWheel_.enableEmergency();
		}
		inline void DisableEmergency(){
			sequence_ = escapeSequence_;
			escapeSequence_ = Sequence::UnInit;
			omuniWheel_.disableEmergency();
		}
		inline void SetSequence(Sequence set_sequence){
			sequence_ = set_sequence;
		}
		inline Sequence GetSequence(){
			return sequence_;
		}
	private:
		std::vector<std::array<float, 2>> passingPointList_;
		std::vector<double> passingAngleList_;
		Sequence sequence_ = Sequence::UnInit;
		//escapesequenseって何？

		Sequence escapeSequence_ = Sequence::UnInit;
		Vector nowVector_ = {.x = 0.0, .y = 0.0, .theta = 0.0};
		Coordinate globalCoordinate_ = {.x = 0.0, .y = 0.0};
		Coordinate targetPoint_ = {.x = 0.0, .y = 0.0};
		RawEachAxisEncoder rawEachAxisEncoder_ = {.x = 0, .y = 0};
		IMUData pIMUData_;

		BNO055 bno055_;
		CatmullRomSpline catmullRomSpline_;
		OmuniWheel omuniWheel_;
		SelfPosition selfPosition_;

		void calVector();
		void SequenceManagement();
		double CalSpeedCoefficient(const double target_meter_per_sec)/*{return (catmullRomSpline_.kChengeVector * 10.0) /}*/;

};



#endif /* MYLIBINC_AUTORUNNING_HPP_ */
