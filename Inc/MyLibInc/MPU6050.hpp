/*
 * MPU6050.hpp
 *
 *  Created on: 2019/06/22
 *      Author: youda
 */

#ifndef MYLIBINC_MPU6050_HPP_
#define MYLIBINC_MPU6050_HPP_

/*
 C++ detection
#ifdef __cplusplus
extern "C" {
#endif
*/

/*
 * 以下は対応したMCUシリーズのヘッダーをインクルードすること
 */
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_i2c.h"

/**
 * @defgroup SD_MPU6050_Macros
 * @brief    Library defines
 * @{
 */

/* Default I2C used */
//#ifndef MPU6050_I2C
//#define	MPU6050_I2C                    I2C1              /*!< Default I2C */
//#define MPU6050_I2C_PINSPACK           SD_I2C_PinsPack_1 /*!< Default I2C pinspack. Check @ref SD_I2C for more information */
//#endif

/* Default I2C clock */
#ifndef MPU6050_I2C_CLOCK
#define MPU6050_I2C_CLOCK              400000            /*!< Default I2C clock speed */
#endif

/**
 * @brief  Data rates predefined constants
 * @{
 */

constexpr uint8_t kMPU6050_DataRate_8KHz =  0;   /*!< Sample rate set to 8 kHz */
constexpr uint8_t kMPU6050_DataRate_4KHz =  1;   /*!< Sample rate set to 4 kHz */
constexpr uint8_t kMPU6050_DataRate_2KHz =  3;   /*!< Sample rate set to 2 kHz */
constexpr uint8_t kMPU6050_DataRate_1KHz =  7;   /*!< Sample rate set to 1 kHz */
constexpr uint8_t kMPU6050_DataRate_500Hz = 15;  /*!< Sample rate set to 500 Hz */
constexpr uint8_t kMPU6050_DataRate_250Hz = 31;  /*!< Sample rate set to 250 Hz */
constexpr uint8_t kMPU6050_DataRate_125Hz = 63;  /*!< Sample rate set to 125 Hz */
constexpr uint8_t kMPU6050_DataRate_100Hz = 79;  /*!< Sample rate set to 100 Hz */

/* Default I2C address */
constexpr uint8_t kMPU6050_I2C_ADDR = 0xD0;

/* Who I am register value */
constexpr uint8_t kMPU6050_I_AM = 0x68;

/* MPU6050 registers */
constexpr uint8_t kMPU6050_AUX_VDDIO = 0x01;
constexpr uint8_t kMPU6050_SMPLRT_DIV = 0x19;
constexpr uint8_t kMPU6050_CONFIG = 0x1A;
constexpr uint8_t kMPU6050_GYRO_CONFIG = 0x1B;
constexpr uint8_t kMPU6050_ACCEL_CONFIG = 0x1C;
constexpr uint8_t kMPU6050_MOTION_THRESH = 0x1F;
constexpr uint8_t kMPU6050_INT_PIN_CFG = 0x37;
constexpr uint8_t kMPU6050_INT_ENABLE = 0x38;
constexpr uint8_t kMPU6050_INT_STATUS = 0x3A;
constexpr uint8_t kMPU6050_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t kMPU6050_ACCEL_XOUT_L = 0x3C;
constexpr uint8_t kMPU6050_ACCEL_YOUT_H = 0x3D;
constexpr uint8_t kMPU6050_ACCEL_YOUT_L = 0x3E;
constexpr uint8_t kMPU6050_ACCEL_ZOUT_H = 0x3F;
constexpr uint8_t kMPU6050_ACCEL_ZOUT_L = 0x40;
constexpr uint8_t kMPU6050_TEMP_OUT_H = 0x41;
constexpr uint8_t kMPU6050_TEMP_OUT_L = 0x42;
constexpr uint8_t kMPU6050_GYRO_XOUT_H = 0x43;
constexpr uint8_t kMPU6050_GYRO_XOUT_L = 0x44;
constexpr uint8_t kMPU6050_GYRO_YOUT_H = 0x45;
constexpr uint8_t kMPU6050_GYRO_YOUT_L = 0x46;
constexpr uint8_t kMPU6050_GYRO_ZOUT_H = 0x47;
constexpr uint8_t kMPU6050_GYRO_ZOUT_L = 0x48;
constexpr uint8_t kMPU6050_MOT_DETECT_STATUS	= 0x61;
constexpr uint8_t kMPU6050_SIGNAL_PATH_RESET	= 0x68;
constexpr uint8_t kMPU6050_MOT_DETECT_CTRL = 0x69;
constexpr uint8_t kMPU6050_USER_CTRL = 0x6A;
constexpr uint8_t kMPU6050_PWR_MGMT_1 = 0x6B;
constexpr uint8_t kMPU6050_PWR_MGMT_2 = 0x6C;
constexpr uint8_t kMPU6050_FIFO_COUNTH = 0x72;
constexpr uint8_t kMPU6050_FIFO_COUNTL = 0x73;
constexpr uint8_t kMPU6050_FIFO_R_W = 0x74;
constexpr uint8_t kMPU6050_WHO_AM_I = 0x75;

/*OffSet value*/
constexpr int16_t kMPU6050_Gyro_X_OffSet  = 110;
constexpr int16_t kMPU6050_Gyro_Y_OffSet  = -600;
constexpr int16_t kMPU6050_Gyro_Z_OffSet  = -80;
constexpr int16_t kMPU6050_Accel_X_OffSet = 300;
constexpr int16_t kMPU6050_Accel_Y_OffSet = 250;
constexpr int16_t kMPU6050_Accel_Z_OffSet = 1350;

/* Gyro sensitivities in degrees/s */
constexpr float kMPU6050_GYRO_SENS_250  = 131.0f;
constexpr float kMPU6050_GYRO_SENS_500  = 65.5f;
constexpr float kMPU6050_GYRO_SENS_1000 = 32.8f;
constexpr float kMPU6050_GYRO_SENS_2000 = 16.4f;

/* Acce sensitivities in g/s */
constexpr float kMPU6050_ACCEL_SENS_2	= 16384.0f;
constexpr float kMPU6050_ACCEL_SENS_4	= 8192.0f;
constexpr float kMPU6050_ACCEL_SENS_8	= 4096.0f;
constexpr float kMPU6050_ACCEL_SENS_16  = 2048.0f;
/**
 * @}
 */

/**
 * @}
 */

/**
 * @defgroup SD_MPU6050_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum  {
	MPU6050_Device_0 = 0x00, /*!< AD0 pin is set to low */
	MPU6050_Device_1 = 0x02  /*!< AD0 pin is set to high */
} MPU6050_Device;

/**
 * @brief  MPU6050 result enumeration
 */
typedef enum  {
	MPU6050_Result_Ok = 0x00,          /*!< Everything OK */
	MPU6050_Result_Error,              /*!< Unknown error */
	MPU6050_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
	MPU6050_Result_DeviceInvalid       /*!< Connected device with address is not MPU6050 */
} MPU6050_Result;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum  {
	MPU6050_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	MPU6050_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	MPU6050_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	MPU6050_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} MPU6050_Accelerometer;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	MPU6050_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	MPU6050_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	MPU6050_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} MPU6050_Gyroscope;

/**
 * @brief  Main MPU6050 structure
 */
typedef struct  {
	/* Private */
	uint8_t Address;         /*!< I2C address of device. */
	float Gyro_Mult;         /*!< Gyroscope corrector from raw data to "degrees/s". Only for private use */
	float Acce_Mult;         /*!< Accelerometer corrector from raw data to "g". Only for private use */
	/* Public */
	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */
	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */
	float   Temperature;       /*!< Temperature in degrees */
	//I2C_HandleTypeDef* I2Cx;
} MPU6050_State;

/**
 * @brief  Interrupts union and structure
 */
typedef union {
	struct {
		uint8_t DataReady:1;       /*!< Data ready interrupt */
		uint8_t reserved2:2;       /*!< Reserved bits */
		uint8_t Master:1;          /*!< Master interrupt. Not enabled with library */
		uint8_t FifoOverflow:1;    /*!< FIFO overflow interrupt. Not enabled with library */
		uint8_t reserved1:1;       /*!< Reserved bit */
		uint8_t MotionDetection:1; /*!< Motion detected interrupt */
		uint8_t reserved0:1;       /*!< Reserved bit */
	} F;
	uint8_t Status;
} MPU6050_Interrupt;

class MPU6050{
public:
	MPU6050(/*I2C_HandleTypeDef* I2Cx,
			MPU6050_State* DataStruct,
			MPU6050_Device DeviceNumber,
			MPU6050_Accelerometer AccelerometerSensitivity,
			MPU6050_Gyroscope GyroscopeSensitivity*/);

	~MPU6050();

/**
	 * @brief  Initializes MPU6050 and I2C peripheral
	 * @param  *DataStruct: Pointer to empty @ref SD_MPU6050_t structure
	 * @param  DeviceNumber: MPU6050 has one pin, AD0 which can be used to set address of device.
	 *          This feature allows you to use 2 different sensors on the same board with same library.
	 *          If you set AD0 pin to low, then this parameter should be SD_MPU6050_Device_0,
	 *          but if AD0 pin is high, then you should use SD_MPU6050_Device_1
	 *
	 *          Parameter can be a value of @ref SD_MPU6050_Device_t enumeration
	 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref SD_MPU6050_Accelerometer_t enumeration
	 * @param  GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref SD_MPU6050_Gyroscope_t enumeration
	 * @retval Initialization status:
	 *            - SD_MPU6050_Result_t: Everything OK
	 *            - Other member: in other cases
	 */
	MPU6050_Result Init(I2C_HandleTypeDef* I2Cx,
				MPU6050_State* DataStruct,
				MPU6050_Device DeviceNumber,
				MPU6050_Accelerometer AccelerometerSensitivity,
				MPU6050_Gyroscope GyroscopeSensitivity);

/**
 * @brief  Enables interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
	MPU6050_Result EnableInterrupts(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct);

/**
 * @brief  Disables interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
	MPU6050_Result DisableInterrupts(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct);

/**
 * @brief  Reads and clears interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @param  *InterruptsStruct: Pointer to @ref SD_MPU6050_Interrupt_t structure to store status in
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
	MPU6050_Result ReadInterrupts(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct, MPU6050_Interrupt* InterruptsStruct);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
	MPU6050_Result ReadAccelerometer(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct);

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
	MPU6050_Result ReadGyroscope(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct);

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
	MPU6050_Result ReadTemperature(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
	MPU6050_Result ReadAll(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

private:


	/**
	 * @brief  Sets gyroscope sensitivity
	 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
	 * @param  GyroscopeSensitivity: Gyro sensitivity value. This parameter can be a value of @ref SD_MPU6050_Gyroscope_t enumeration
	 * @retval Member of @ref SD_MPU6050_Result_t enumeration
	 */
		MPU6050_Result SetGyroscope(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct, MPU6050_Gyroscope GyroscopeSensitivity);

	/**
	 * @brief  Sets accelerometer sensitivity
	 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
	 * @param  AccelerometerSensitivity: Gyro sensitivity value. This parameter can be a value of @ref SD_MPU6050_Accelerometer_t enumeration
	 * @retval Member of @ref SD_MPU6050_Result_t enumeration
	 */
		MPU6050_Result SetAccelerometer(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct, MPU6050_Accelerometer AccelerometerSensitivity);

	/**
	 * @brief  Sets output data rate
	 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
	 * @param  rate: Data rate value. An 8-bit value for prescaler value
	 * @retval Member of @ref SD_MPU6050_Result_t enumeration
	 */
		MPU6050_Result SetDataRate(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct, uint8_t rate);
};


#endif /* MYLIBINC_MPU6050_HPP_ */
