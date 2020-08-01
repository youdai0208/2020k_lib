/*
 * BNO055.hpp
 *
 *  Created on: 2019/07/15
 *      Author: youda
 */

#ifndef MYLIBINC_BNO055_HPP_
#define MYLIBINC_BNO055_HPP_

#include <cmath>

#include "stm32f3xx_hal.h"

class BNO055{
public:
	enum class system_status_t {
	  SYSTEM_STATUS_IDLE = 0x00,
	  SYSTEM_STATUS_SYSTEM_ERROR = 0x01,
	  SYSTEM_STATUS_INITIALIZING_PERIPHERALS = 0x02,
	  SYSTEM_STATUS_SYSTEM_INITIALIZATION = 0x03,
	  SYSTEM_STATUS_EXECUTING_SELF_TEST = 0x04,
	  SYSTEM_STATUS_FUSION_ALGO_RUNNING = 0x05,
	  SYSTEM_STATUS_FUSION_ALOG_NOT_RUNNING = 0x06
	};

	enum class opmode_t {  // BNO-55 operation modes
	  OPERATION_MODE_CONFIG = 0x00,
	  // Sensor Mode
	  OPERATION_MODE_ACCONLY,
	  OPERATION_MODE_MAGONLY,
	  OPERATION_MODE_GYRONLY,
	  OPERATION_MODE_ACCMAG,
	  OPERATION_MODE_ACCGYRO,
	  OPERATION_MODE_MAGGYRO,
	  OPERATION_MODE_AMG,  // 0x07
	                              // Fusion Mode
	  OPERATION_MODE_IMU,
	  OPERATION_MODE_COMPASS,
	  OPERATION_MODE_M4G,
	  OPERATION_MODE_NDOF_FMC_OFF,
	  OPERATION_MODE_NDOF  // 0x0C
	};

	enum class vector_type_t {
	  VECTOR_ACCELEROMETER = 0x08,  // Default: m/s²
	  VECTOR_MAGNETOMETER = 0x0E,   // Default: uT
	  VECTOR_GYROSCOPE = 0x14,      // Default: rad/s
	  VECTOR_EULER = 0x1A,          // Default: degrees
	  VECTOR_QUATERNION = 0x20,
	  VECTOR_LINEARACCEL = 0x28,    // Default: m/s²
	  VECTOR_GRAVITY = 0x2E         // Default: m/s²
	};

	enum class system_error_t {
	  SYSTEM_ERROR_NO_ERROR = 0x00,
	  SYSTEM_ERROR_PERIPHERAL_INITIALIZATION_ERROR = 0x01,
	  SYSTEM_ERROR_SYSTEM_INITIALIZATION_ERROR = 0x02,
	  SYSTEM_ERROR_SELF_TEST_FAILED = 0x03,
	  SYSTEM_ERROR_REG_MAP_VAL_OUT_OF_RANGE = 0x04,
	  SYSTEM_ERROR_REG_MAP_ADDR_OUT_OF_RANGE = 0x05,
	  SYSTEM_ERROR_REG_MAP_WRITE_ERROR = 0x06,
	  SYSTEM_ERROR_LOW_PWR_MODE_NOT_AVAILABLE_FOR_SELECTED_OPR_MODE = 0x07,
	  SYSTEM_ERROR_ACCEL_PWR_MODE_NOT_AVAILABLE = 0x08,
	  SYSTEM_ERROR_FUSION_ALGO_CONF_ERROR = 0x09,
	  SYSTEM_ERROR_SENSOR_CONF_ERROR = 0x0A
	};

	struct self_test_result_t {
	  uint8_t mcuState;
	  uint8_t gyrState;
	  uint8_t magState;
	  uint8_t accState;
	} ;

	struct calibration_t {
	  uint8_t sys;
	  uint8_t gyro;
	  uint8_t mag;
	  uint8_t accel;
	} ;

	struct vector_t {
	  double x;
	  double y;
	  double z;
	};

	struct quaternion_t {
	  double w;
	  double x;
	  double y;
	  double z;
	};

	BNO055();
	~BNO055();

	void Reset();
	void SetOperationMode(const opmode_t mode);
	void SetOperationModeConfig();
	void SetOperationModeNDOF();
	void Setup();
	void AssignI2C(I2C_HandleTypeDef *hi2c_device);

	int8_t GetTemp();

	uint8_t GetBootloaderRevision();
	uint8_t GetSystemStatus();
	uint8_t GetSystemError();
	int16_t GetSWRevision();

	self_test_result_t GetSelfTestResult();
	calibration_t GetCalibration();
	vector_t GetVectorAccelerometer();
	vector_t GetVectorMagnetometer();
	vector_t GetVectorGyroscope();
	vector_t GetVectorEuler();
	quaternion_t GetQuaternion();
	vector_t GetVectorLinearAccel();
	vector_t GetVectorGravity();
	vector_t QuaternionToEuler(const quaternion_t quaternion);

private:
	static constexpr uint8_t kStartByte 				= 0xAA;
	static constexpr uint8_t kResponseByte  			= 0xBB;
	static constexpr uint8_t kErrorByte 				= 0xEE;
	static constexpr uint8_t kI2CAddrHi 				= 0x29;
	static constexpr uint8_t kI2CAddrLo 				= 0x28;
	static constexpr uint8_t kErrorWriteSuccese 		= 0x01;
	static constexpr uint8_t kErrorWriteFail 			= 0x03;
	static constexpr uint8_t kErrorRegmapInvAddr    	= 0x04;
	static constexpr uint8_t kErrorRegmapWriteDis 		= 0x05;
	static constexpr uint8_t kErrorWrongStartByte 		= 0x06;
	static constexpr uint8_t kErrorBusOverrunErr 		= 0x07;
	static constexpr uint8_t kErrorMaxLenErr 			= 0x08;
	static constexpr uint8_t kErrorMinLenErr 			= 0x09;
	static constexpr uint8_t kErrorRecvCharTimeout 		= 0x0A;
	static constexpr uint8_t kRegWrite	 				= 0x00;
	static constexpr uint8_t kRegRead 					= 0x01;
	static constexpr uint32_t kReadTimeout	 			= 100;
	static constexpr uint32_t kWriteTimeout 			= 100;

	static constexpr double kaccelScale 				= 100.0;
	static constexpr double ktempScale 					= 1.0;
	static constexpr double kangularRateScale 			= 16.0;
	static constexpr double keulerScale 				= 16.0;
	static constexpr double kmagScale 					= 16.0;
	static constexpr double kquaternionScale 			= 16384.0;

	enum class Register1 {
		ID = 0xA0,
		CHIP_ID = 0x00,
		ACC_ID,
		MAG_ID,
		GYRO_ID,
		SW_REV_ID_LSB,
		SW_REV_ID_MSB,
		BL_REV_ID,
		PAGE_ID,
		ACC_DATA_X_LSB,
		ACC_DATA_X_MSB,
		ACC_DATA_Y_LSB,
		ACC_DATA_Y_MSB,
		ACC_DATA_Z_LSB,
		ACC_DATA_Z_MSB,
		MAG_DATA_X_LSB,
		MAG_DATA_X_MSB,
		MAG_DATA_Y_LSB,
		MAG_DATA_Y_MSB,
		MAG_DATA_Z_LSB,
		MAG_DATA_Z_MSB,
		GYR_DATA_X_LSB,
		GYR_DATA_X_MSB,
		GYR_DATA_Y_LSB,
		GYR_DATA_Y_MSB,
		GYR_DATA_Z_LSB,
		GYR_DATA_Z_MSB,
		EUL_HEADING_LSB,
		EUL_HEADING_MSB,
		EUL_ROLL_LSB,
		EUL_ROLL_MSB,
		EUL_PITCH_LSB,
		EUL_PITCH_MSB,
		QUA_DATA_W_LSB,
		QUA_DATA_W_MSB,
		QUA_DATA_X_LSB,
		QUA_DATA_X_MSB,
		QUA_DATA_Y_LSB,
		QUA_DATA_Y_MSB,
		QUA_DATA_Z_LSB,
		QUA_DATA_Z_MSB,
		LIA_DATA_X_LSB,
		LIA_DATA_X_MSB,
		LIA_DATA_Y_LSB,
		LIA_DATA_Y_MSB,
		LIA_DATA_Z_LSB,
		LIA_DATA_Z_MSB,
		GRV_DATA_X_LSB,
		GRV_DATA_X_MSB,
		GRV_DATA_Y_LSB,
		GRV_DATA_Y_MSB,
		GRV_DATA_Z_LSB,
		GRV_DATA_Z_MSB,
		TEMP_DATA,
		CALIB_STAT,
		ST_RESULT,
		INT_STA,
		SYS_CLK_STATUS,
		SYS_STATUS,
		SYS_ERR,
		UNIT_SEL,
		OPR_MODE = 0x3D,
		PWR_MODE,
		SYS_TRIGGER,
		TEMP_SOURCE,
		AXIS_MAP_CONFIG,
		AXIS_MAP_SIGN,
		ACC_OFFSET_X_LSB = 0x55,
		ACC_OFFSET_X_MSB,
		ACC_OFFSET_Y_LSB,
		ACC_OFFSET_Y_MSB,
		ACC_OFFSET_Z_LSB,
		ACC_OFFSET_Z_MSB,
		MAG_OFFSET_X_LSB,
		MAG_OFFSET_X_MSB,
		MAG_OFFSET_Y_LSB,
		MAG_OFFSET_Y_MSB,
		MAG_OFFSET_Z_LSB,
		MAG_OFFSET_Z_MSB,
		GYR_OFFSET_X_LSB,
		GYR_OFFSET_X_MSB,
		GYR_OFFSET_Y_LSB,
		GYR_OFFSET_Y_MSB,
		GYR_OFFSET_Z_LSB,
		GYR_OFFSET_Z_MSB,
		ACC_RADIUS_LSB,
		ACC_RADIUS_MSB,
		MAG_RADIUS_LSB,
		MAG_RADIUS_MSB
	};

	enum class Register2 {
		PAGE_ID = 0x07,
		ACC_CONFIG,
		MAG_CONFIG,
		GYRO_CONFIG_0,
		GYRO_CONFIG_1,
		ACC_SLEEP_CONFIG,
		GYR_SLEEP_CONFIG,
		INT_MSK = 0x0F,
		INT_EN,
		ACC_AM_THRES,
		ACC_INT_SETTINGS,
		ACC_HG_DURATION,
		ACC_HG_THRESH,
		ACC_NM_THRESH,
		ACC_NM_SET,
		GYR_INT_SETTINGS,
		GYR_HR_X_SET,
		GYR_DUR_X,
		GYR_HR_Y_SET,
		GYR_DUR_Y,
		GYR_HR_Z_SET,
		GYR_DUR_Z,
		GYR_AM_THRESH,
		GYR_AM_SET
	};

	I2C_HandleTypeDef *i2c_handle_;

	void SetPage(const uint8_t page);
	void Delay(const int time);
	void EnableExternalCrystal();
	void DisableExternalCrystal();
	void WriteData(const Register1 reg, const uint8_t data);
	void WriteData(const Register1 reg, const opmode_t data);
	void WriteData(const vector_type_t vec, const uint8_t data);
	void WriteData(const uint8_t reg, const uint8_t data);
	void ReadData(Register1 reg, uint8_t* data, uint8_t len);
	void ReadData(vector_type_t reg, uint8_t* data, uint8_t len);
	void ReadData(uint8_t reg, uint8_t* data, uint8_t len);
	void SetExternalCrystalUse(const bool state);
	vector_t GetVector(const vector_type_t vec);
};



#endif /* MYLIBINC_BNO055_HPP_ */
