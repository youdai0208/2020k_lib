/*
 * MPU6050.cpp
 *
 *  Created on: 2019/06/22
 *      Author: youda
 */

#include "MyLibInc/MPU6050.hpp"

MPU6050::MPU6050(/*I2C_HandleTypeDef* I2Cx,
		MPU6050_State* DataStruct,
		MPU6050_Device DeviceNumber,
		MPU6050_Accelerometer AccelerometerSensitivity,
		MPU6050_Gyroscope GyroscopeSensitivity*/){
	/*if(this->Init(I2Cx, DataStruct, DeviceNumber, AccelerometerSensitivity, GyroscopeSensitivity) != MPU6050_Result_Ok){

		while(1){

		}
	}*/
}

MPU6050::~MPU6050(){

}

MPU6050_Result MPU6050::Init(I2C_HandleTypeDef* I2Cx,
		MPU6050_State* DataStruct,
		MPU6050_Device DeviceNumber,
		MPU6050_Accelerometer AccelerometerSensitivity,
		MPU6050_Gyroscope GyroscopeSensitivity)
{
	uint8_t WHO_AM_I = (uint8_t)kMPU6050_WHO_AM_I;
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t d[2];

	/* Format I2C address */
	DataStruct->Address = kMPU6050_I2C_ADDR | (uint8_t)DeviceNumber;
	uint8_t address = DataStruct->Address;

	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(Handle,address,2,5)!=HAL_OK)
	{
				return MPU6050_Result_Error;
	}
	/* Check who am I */
	//------------------
		/* Send address */
		if(HAL_I2C_Master_Transmit(Handle, address, &WHO_AM_I, 1, 1000) != HAL_OK)
		{
			return MPU6050_Result_Error;
		}

		/* Receive multiple byte */
		if(HAL_I2C_Master_Receive(Handle, address, &temp, 1, 1000) != HAL_OK)
		{
			return MPU6050_Result_Error;
		}

		/* Checking */
		while(temp != kMPU6050_I_AM)
		{
				/* Return error */
				return MPU6050_Result_DeviceInvalid;
		}
	//------------------

	/* Wakeup MPU6050 */
	//------------------
		/* Format array to send */
		d[0] = kMPU6050_PWR_MGMT_1;
		d[1] = 0x00;

		/* Try to transmit via I2C */
		if(HAL_I2C_Master_Transmit(Handle,(uint16_t)address , (uint8_t *)d, 2, 1000) != HAL_OK)
		{
					return MPU6050_Result_Error;
		}
	//------------------

	/* Set sample rate to 1kHz */
	this->SetDataRate(I2Cx,DataStruct, kMPU6050_DataRate_100Hz);

	/* Config accelerometer */
	this->SetAccelerometer(I2Cx,DataStruct, AccelerometerSensitivity);

	/* Config Gyroscope */
	this->SetGyroscope(I2Cx,DataStruct, GyroscopeSensitivity);

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050::SetDataRate(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct, uint8_t rate)
{
	uint8_t d[2];
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	/* Format array to send */
	d[0] = kMPU6050_SMPLRT_DIV;
	d[1] = rate;

	/* Set data sample rate */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)address,(uint8_t *)d,2,1000)!=HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050::SetAccelerometer(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct, MPU6050_Accelerometer AccelerometerSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	uint8_t regAdd =(uint8_t )kMPU6050_ACCEL_CONFIG;

	/* Config accelerometer */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &regAdd, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/

	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case MPU6050_Accelerometer_2G:
			DataStruct->Acce_Mult = (float)1 / kMPU6050_ACCEL_SENS_2;
			break;
		case MPU6050_Accelerometer_4G:
			DataStruct->Acce_Mult = (float)1 / kMPU6050_ACCEL_SENS_4;
			break;
		case MPU6050_Accelerometer_8G:
			DataStruct->Acce_Mult = (float)1 / kMPU6050_ACCEL_SENS_8;
			break;
		case MPU6050_Accelerometer_16G:
			DataStruct->Acce_Mult = (float)1 / kMPU6050_ACCEL_SENS_16;
			break;
		default:
			break;
		}

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050::SetGyroscope(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct, MPU6050_Gyroscope GyroscopeSensitivity)
{
	uint8_t temp;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;
	uint8_t regAdd =(uint8_t )kMPU6050_GYRO_CONFIG;

	/* Config gyroscope */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &regAdd, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address,&temp, 1, 1000) != HAL_OK);
	/*{
				return SD_MPU6050_Result_Error;
	}*/

	switch (GyroscopeSensitivity) {
			case MPU6050_Gyroscope_250s:
				DataStruct->Gyro_Mult = (float)1 / kMPU6050_GYRO_SENS_250;
				break;
			case MPU6050_Gyroscope_500s:
				DataStruct->Gyro_Mult = (float)1 / kMPU6050_GYRO_SENS_500;
				break;
			case MPU6050_Gyroscope_1000s:
				DataStruct->Gyro_Mult = (float)1 / kMPU6050_GYRO_SENS_1000;
				break;
			case MPU6050_Gyroscope_2000s:
				DataStruct->Gyro_Mult = (float)1 / kMPU6050_GYRO_SENS_2000;
				break;
			default:
				break;
		}
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050::ReadAccelerometer(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct)
{
	uint8_t data[6];
	uint8_t reg = (uint8_t )kMPU6050_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read accelerometer data */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 6, 1000) != HAL_OK);

	/* Format */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050::ReadGyroscope(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct)
{
	uint8_t data[6];
	uint8_t reg = (uint8_t )kMPU6050_GYRO_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read gyroscope data */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 6, 1000) != HAL_OK);

	/* Format */
	DataStruct->Gyroscope_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Gyroscope_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Gyroscope_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050::ReadTemperature(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct)
{
	uint8_t data[2];
	int16_t temp;
//	uint8_t reg = kMPU6050_TEMP_OUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read temperature */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, (uint8_t *)kMPU6050_TEMP_OUT_H, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 2, 1000) != HAL_OK);

	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050::ReadAll(I2C_HandleTypeDef* I2Cx,MPU6050_State* DataStruct)
{
	uint8_t data[14];
	int16_t temp;
//	uint8_t reg = MPU6050_ACCEL_XOUT_H;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Read full raw data, 14bytes */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, (uint8_t *)kMPU6050_ACCEL_XOUT_H, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, data, 14, 1000) != HAL_OK);

	/* Format accelerometer data */
	DataStruct->Accelerometer_X = (int16_t)(data[0] << 8 | data[1]);
	DataStruct->Accelerometer_Y = (int16_t)(data[2] << 8 | data[3]);
	DataStruct->Accelerometer_Z = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	DataStruct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

	/* Format gyroscope data */
	DataStruct->Gyroscope_X = (int16_t)(data[8] << 8 | data[9]);
	DataStruct->Gyroscope_Y = (int16_t)(data[10] << 8 | data[11]);
	DataStruct->Gyroscope_Z = (int16_t)(data[12] << 8 | data[13]);

	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050::EnableInterrupts(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct)
{
	uint8_t temp;
	uint8_t reg[2] = {kMPU6050_INT_ENABLE,0x21};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Enable interrupts for data ready and motion detect */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	uint8_t mpu_reg= kMPU6050_INT_PIN_CFG;
	/* Clear IRQ flag on any read operation */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &mpu_reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 14, 1000) != HAL_OK);
	temp |= 0x10;
	reg[0] = kMPU6050_INT_PIN_CFG;
	reg[1] = temp;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050::DisableInterrupts(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct)
{
	uint8_t reg[2] = {kMPU6050_INT_ENABLE,0x00};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Disable interrupts */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)address,reg,2,1000)!=HAL_OK);
	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050::ReadInterrupts(I2C_HandleTypeDef* I2Cx, MPU6050_State* DataStruct, MPU6050_Interrupt* InterruptsStruct)
{
	uint8_t read;

	/* Reset structure */
	InterruptsStruct->Status = 0;
	uint8_t reg = (uint8_t )kMPU6050_INT_STATUS;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &read, 14, 1000) != HAL_OK);

	/* Fill value */
	InterruptsStruct->Status = read;
	/* Return OK */
	return MPU6050_Result_Ok;
}


