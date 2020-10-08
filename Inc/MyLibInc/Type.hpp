/*
 * Type.hpp
 *
 *  Created on: 2019/05/31
 *      Author: youda
 */

#ifndef MYLIBINC_TYPE_HPP_
#define MYLIBINC_TYPE_HPP_

struct State {
	double x;
	double y;
	double yaw;
};

struct Vector {
	/* data */
	double x;
	double y;
	double theta;
};

struct Coordinate {
	/* data */
	double x;
	double y;
};

struct PIDResult{
	double x;
	double y;
	double theta;
};

struct EachAxisEncoder {
	double x;
	double y;
};

struct EachAxisEncoderWheelOdometry {
	double encoder1;
	double encoder2;
};

struct RawEachAxisEncoder {
	uint32_t x;
	uint32_t y;
};

/*struct EachAxisPosition {
	double x;
	double y;
};*/

struct AmountOfMovement {
	double x;
	double y;
};

struct GlobalCoordinates {
	double x;
	double y;
};

struct RawIMUData{
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
};

struct IMUData {
	double accel_x;
	double accel_y;
	double accel_z;
	double gyro_x;
	double gyro_y;
	double gyro_z;
	double roll;
	double pitch;
	double yaw;
};

#endif /* MYLIBINC_TYPE_HPP_ */
