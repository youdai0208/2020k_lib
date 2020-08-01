/*
 * BNO055.cpp
 *
 *  Created on: 2019/07/15
 *      Author: youda
 */

//#include "MyLibInc/BNO055.hpp"
#include "MyLibInc/BNO055.hpp"

BNO055::BNO055(){

}

BNO055::~BNO055(){
	i2c_handle_ = NULL;
}

void BNO055::SetPage(const uint8_t page){
	WriteData(Register1::PAGE_ID, page);
}

void BNO055::Reset(){
	WriteData(static_cast<uint8_t>(Register1::SYS_TRIGGER), 0x20);
//	Delay(700);
}

void BNO055::Delay(const int time){
	HAL_Delay(time);
}

void BNO055::SetOperationMode(const opmode_t mode){
	WriteData(Register1::OPR_MODE, mode);
	Delay(50);
}

void BNO055::SetOperationModeConfig(){
	SetOperationMode(opmode_t::OPERATION_MODE_CONFIG);
}

void BNO055::SetOperationModeNDOF(){
	SetOperationMode(opmode_t::OPERATION_MODE_NDOF);
}

void BNO055::SetExternalCrystalUse(const bool state){
	uint8_t tmp = 0;
	SetPage(0);

	ReadData(Register1::SYS_TRIGGER, &tmp, 1);
	tmp |= (state == true) ? 0x80 : 0x0;
	WriteData(Register1::SYS_TRIGGER, tmp);
	Delay(700);
}

void BNO055::EnableExternalCrystal(){
	SetExternalCrystalUse(true);
}

void BNO055::DisableExternalCrystal(){
	SetExternalCrystalUse(false);
}

void BNO055::AssignI2C(I2C_HandleTypeDef *hi2c_device){
	i2c_handle_ = hi2c_device;
}

void BNO055::Setup(){
	uint8_t id = 0;
	Reset();

	ReadData(Register1::CHIP_ID, &id, 1);
	if(id != static_cast<uint8_t>(Register1::ID)){
		printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n", id);
	}
	SetPage(0);
	WriteData(Register1::SYS_TRIGGER, 0x0);

	SetOperationModeConfig();
	Delay(10);
}

int8_t BNO055::GetTemp(){
	uint8_t t = 0;
	SetPage(0);
	ReadData(Register1::TEMP_DATA, &t, 1);
	return t;
}

uint8_t BNO055::GetBootloaderRevision(){
	uint8_t tmp;
	SetPage(0);
	ReadData(Register1::BL_REV_ID, &tmp, 1);
	return tmp;
}

uint8_t BNO055::GetSystemStatus(){
	uint8_t tmp;
	SetPage(0);
	ReadData(Register1::SYS_STATUS, &tmp, 1);
	return tmp;
}

uint8_t BNO055::GetSystemError(){
	uint8_t tmp;
	SetPage(0);
	ReadData(Register1::SYS_ERR, &tmp, 1);
	return tmp;
}

int16_t BNO055::GetSWRevision(){
	uint8_t buffer[2];
	SetPage(0);
	ReadData(Register1::SW_REV_ID_LSB, buffer, 2);
	return static_cast<int16_t>(((buffer[1] << 8) | buffer[0]));
}


BNO055::self_test_result_t BNO055::GetSelfTestResult(){
	uint8_t tmp;
	self_test_result_t res = {
	    .mcuState = 0, .gyrState = 0, .magState = 0, .accState = 0};
	SetPage(0);
	ReadData(Register1::ST_RESULT, &tmp, 1);
	res.mcuState = (tmp >> 3) & 0x01;
	res.gyrState = (tmp >> 2) & 0x01;
	res.magState = (tmp >> 1) & 0x01;
	res.accState = (tmp >> 0) & 0x01;
	return res;
}

BNO055::calibration_t BNO055::GetCalibration(){
	calibration_t cal = {.sys = 0, .gyro = 0, .mag = 0, .accel = 0};
	uint8_t calData = 0;
	SetPage(0);
	ReadData(Register1::CALIB_STAT, &calData, 1);
	cal.sys = (calData >> 6) & 0x03;
	cal.gyro = (calData >> 4) & 0x03;
	cal.accel = (calData >> 2) & 0x03;
	cal.mag = calData & 0x03;
	return cal;
}

BNO055::vector_t BNO055::GetVectorAccelerometer(){
	return GetVector(vector_type_t::VECTOR_ACCELEROMETER);
}

BNO055::vector_t BNO055::GetVectorMagnetometer(){
	return GetVector(vector_type_t::VECTOR_MAGNETOMETER);
}

BNO055::vector_t BNO055::GetVectorGyroscope(){
	return GetVector(vector_type_t::VECTOR_GYROSCOPE);
}

BNO055::vector_t BNO055::GetVectorEuler(){
	return GetVector(vector_type_t::VECTOR_EULER);
}

BNO055::vector_t BNO055::GetVectorLinearAccel(){
	return GetVector(vector_type_t::VECTOR_LINEARACCEL);
}

BNO055::vector_t BNO055::GetVectorGravity(){
	return GetVector(vector_type_t::VECTOR_GRAVITY);
}

BNO055::quaternion_t BNO055::GetQuaternion(){
	uint8_t buffer[8] = {0};
	quaternion_t quaternion;
//	double scale = kquaternionScale;

	ReadData(vector_type_t::VECTOR_QUATERNION, buffer, 8);

	quaternion.w = ((int16_t)((buffer[1] << 8) | buffer[0])) / kquaternionScale;
	quaternion.x = ((int16_t)((buffer[3] << 8) | buffer[2])) / kquaternionScale;
	quaternion.y = ((int16_t)((buffer[5] << 8) | buffer[4])) / kquaternionScale;
	quaternion.z = ((int16_t)((buffer[7] << 8) | buffer[6])) / kquaternionScale;

	return quaternion;
}

BNO055::vector_t BNO055::QuaternionToEuler(const quaternion_t quaternion){
	vector_t cal_result;
	double t0, t1, t2, t3, t4;

	t0 = +2.0 * ((quaternion.w * quaternion.x) + (quaternion.y * quaternion.z));
	t1 = +1.0 - (2.0 * ((quaternion.x * quaternion.x) + (quaternion.y * quaternion.y)));
	cal_result.x = atan2(t0, t1) * 57.2957795131;

	t2 = +2.0 * ((quaternion.w * quaternion.y) - (quaternion.z * quaternion.x));
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	cal_result.y = asin(t2) * 57.2957795131;

	t3 = +2.0 * ((quaternion.w * quaternion.z) + (quaternion.x * quaternion.y));
	t4 = +1.0 - (2.0 * ((quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)));
	cal_result.z = atan2(t3, t4) * 57.2957795131;

	return cal_result;
}

void BNO055::WriteData(const Register1 reg, const uint8_t data){
	WriteData(static_cast<uint8_t>(reg), data);
}

void BNO055::WriteData(const Register1 reg, const opmode_t data){
	WriteData(static_cast<uint8_t>(reg), static_cast<uint8_t>(data));
}

void BNO055::WriteData(const vector_type_t reg, const uint8_t data){
	WriteData(static_cast<uint8_t>(reg), data);
}

void BNO055::WriteData(const uint8_t reg, const uint8_t data){
	uint8_t txdata[2] = {reg, data};
	uint8_t status;
	status = HAL_I2C_Master_Transmit(i2c_handle_, kI2CAddrLo << 1,
	                                   txdata, sizeof(txdata), kWriteTimeout);
	if (status == HAL_OK) {
	  return;
	}

	if (status == HAL_ERROR) {
	  printf("HAL_I2C_Master_Transmit HAL_ERROR\r\n");
	} else if (status == HAL_TIMEOUT) {
	  printf("HAL_I2C_Master_Transmit HAL_TIMEOUT\r\n");
	} else if (status == HAL_BUSY) {
	  printf("HAL_I2C_Master_Transmit HAL_BUSY\r\n");
	} else {
	  printf("Unknown status data %d", status);
	}

	uint32_t error = HAL_I2C_GetError(i2c_handle_);
	if (error == HAL_I2C_ERROR_NONE) {
	  return;
	} else if (error == HAL_I2C_ERROR_BERR) {
	  printf("HAL_I2C_ERROR_BERR\r\n");
	} else if (error == HAL_I2C_ERROR_ARLO) {
	  printf("HAL_I2C_ERROR_ARLO\r\n");
	} else if (error == HAL_I2C_ERROR_AF) {
	  printf("HAL_I2C_ERROR_AF\r\n");
	} else if (error == HAL_I2C_ERROR_OVR) {
	  printf("HAL_I2C_ERROR_OVR\r\n");
	} else if (error == HAL_I2C_ERROR_DMA) {
	  printf("HAL_I2C_ERROR_DMA\r\n");
	} else if (error == HAL_I2C_ERROR_TIMEOUT) {
	  printf("HAL_I2C_ERROR_TIMEOUT\r\n");
	}

	HAL_I2C_StateTypeDef state = HAL_I2C_GetState(i2c_handle_);
	if (state == HAL_I2C_STATE_RESET) {
	  printf("HAL_I2C_STATE_RESET\r\n");
	} else if (state == HAL_I2C_STATE_READY) {
	  printf("HAL_I2C_STATE_RESET\r\n");
	} else if (state == HAL_I2C_STATE_BUSY) {
	  printf("HAL_I2C_STATE_BUSY\r\n");
	} else if (state == HAL_I2C_STATE_BUSY_TX) {
	  printf("HAL_I2C_STATE_BUSY_TX\r\n");
	} else if (state == HAL_I2C_STATE_BUSY_RX) {
	  printf("HAL_I2C_STATE_BUSY_RX\r\n");
	} else if (state == HAL_I2C_STATE_LISTEN) {
	  printf("HAL_I2C_STATE_LISTEN\r\n");
	} else if (state == HAL_I2C_STATE_BUSY_TX_LISTEN) {
	  printf("HAL_I2C_STATE_BUSY_TX_LISTEN\r\n");
	} else if (state == HAL_I2C_STATE_BUSY_RX_LISTEN) {
	  printf("HAL_I2C_STATE_BUSY_RX_LISTEN\r\n");
	} else if (state == HAL_I2C_STATE_ABORT) {
	  printf("HAL_I2C_STATE_ABORT\r\n");
	} else if (state == HAL_I2C_STATE_TIMEOUT) {
	  printf("HAL_I2C_STATE_TIMEOUT\r\n");
	} else if (state == HAL_I2C_STATE_ERROR) {
	  printf("HAL_I2C_STATE_ERROR\r\n");
	}

}

void BNO055::ReadData( Register1 reg, uint8_t* data, uint8_t len){
	uint8_t send_data = static_cast<uint8_t>(reg);
	ReadData(send_data, data, len);
//	uint16_t address = static_cast<uint16_t>(kI2C_ADDR_LO << 1);
//	while(HAL_I2C_Master_Transmit(i2c_handle_, address, &send_data, 1, 1000) != HAL_OK);
//	while(HAL_I2C_Master_Receive(i2c_handle_, address, data, len, 1000) != HAL_OK);
}

void BNO055::ReadData(vector_type_t reg, uint8_t* data, uint8_t len){
	uint8_t send_data = static_cast<uint8_t>(reg);
	ReadData(send_data, data, len);
//	uint16_t address = static_cast<uint16_t>(kI2C_ADDR_LO << 1);
//	while(HAL_I2C_Master_Transmit(&i2c_handle_, address, &send_data, 1, 1000) != HAL_OK);
//	while(HAL_I2C_Master_Receive(&i2c_handle_, address, data, len, 1000) != HAL_OK);
}

void BNO055::ReadData(uint8_t reg, uint8_t* data, uint8_t len){
	uint16_t address = static_cast<uint16_t>(kI2CAddrLo << 1);
	while(HAL_I2C_Master_Transmit(i2c_handle_, kI2CAddrLo << 1, &reg, 1, kWriteTimeout) != HAL_OK);
	while(HAL_I2C_Master_Receive(i2c_handle_, address, data, len, kReadTimeout) != HAL_OK);
}

BNO055::vector_t BNO055::GetVector(const vector_type_t vec){
	 vector_t xyz = {.x = 0, .y = 0, .z = 0};
	 uint8_t buffer[6];
	 double scale = 1.0;
	 SetPage(0);

	 ReadData(vec, buffer, 6);

	 if (vec == vector_type_t::VECTOR_MAGNETOMETER) {
	   scale = kmagScale;
	 } else if (vec == vector_type_t::VECTOR_ACCELEROMETER ||
	    vec == vector_type_t::VECTOR_LINEARACCEL || vec == vector_type_t::VECTOR_GRAVITY) {
	   scale = kaccelScale;
	 } else if (vec == vector_type_t::VECTOR_GYROSCOPE) {
	   scale = kangularRateScale;
	 } else if (vec == vector_type_t::VECTOR_EULER) {
	   scale = keulerScale;
	 }

	 xyz.x = (int16_t)((buffer[1] << 8) | buffer[0]) / scale;
	 xyz.y = (int16_t)((buffer[3] << 8) | buffer[2]) / scale;
	 xyz.z = (int16_t)((buffer[5] << 8) | buffer[4]) / scale;

	 return xyz;
}


