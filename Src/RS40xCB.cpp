
/*
 * RS40xCB.cpp
 *
 *  Created on: 2019/07/29
 *      Author: tench
 */
#include <MyLibInc/RS40xCB.hpp>


RS40xCB::RS40xCB(IOPin io) : servoWriteRead_(io){

}

RS40xCB::~RS40xCB(){

}

void RS40xCB::Init(UART_HandleTypeDef* uarthandle){
	uart_ = uarthandle;
}

void RS40xCB::SendDataOfShortpacket(uint8_t id, memory_adr address, std::vector<uint8_t> data){
	std::vector<uint8_t> packet;
	packet.reserve(8 + data.size());

	/*header*/
	packet.push_back(kHeader_HighByte);
	packet.push_back(kHeader_LowByte);

	/*servo_id*/
	packet.push_back(id);
	/*flags*/
	packet.push_back(0);
	/*address*/
	packet.push_back(static_cast<uint8_t>(address));
	/*length*/
	packet.push_back(static_cast<uint8_t>(data.size()));
	/*count*/
	packet.push_back(1);
	/*data*/
	for(auto d : data)packet.push_back(d);

	/*sum*/
	packet.push_back(0);
	for(uint8_t i = 2;i < (packet.size() - 1);++i)
		packet.back() ^= packet.at(i);


	servoWriteRead_.write(true);
	HAL_UART_Transmit(uart_, packet.data(), static_cast<uint16_t>(packet.size()), 100);
	servoWriteRead_.write(false);

}


void RS40xCB::SendDataOfLongpacket(const std::vector<uint8_t> id,const uint8_t address, const std::vector<std::vector<uint8_t>> data){
	std::vector<uint8_t> packet;
	uint8_t length = static_cast<uint8_t>(data.at(0).size() + 1U);
	uint8_t data_count = 0;
	packet.reserve(8 + length * id.size());

	/*header*/
	packet.push_back(kHeader_HighByte);
	packet.push_back(kHeader_LowByte);

	/*servo id*/
	packet.push_back(0);
	/*flags*/
	packet.push_back(0);
	/*address*/
	packet.push_back(address);
	/*length*/
	packet.push_back(length);
	/*count*/
	packet.push_back(static_cast<uint8_t>(id.size()));

	/*data*/
	for(auto servo : id){
		packet.push_back(servo);
		for(auto d : data.at(data_count))packet.push_back(d);
		data_count++;
	}
	/*sum*/
	packet.push_back(0);
	for(uint8_t i = 2;i < (packet.size() - 1);++i)
		packet.back() ^= packet.at(i);

	servoWriteRead_.write(true);
//	HAL_Delay(1);
	HAL_UART_Transmit(uart_, packet.data(), static_cast<uint16_t>(packet.size()), 100);
//	HAL_Delay(1);
	servoWriteRead_.write(false);
}

void RS40xCB::SpecialTransmissionData(uint8_t id, flags flag, memory_adr address){
	std::vector<uint8_t> packet;
	packet.reserve(8);

	/*header*/
	packet.push_back(kHeader_HighByte);
	packet.push_back(kHeader_LowByte);

	/*servo id*/
	packet.push_back(id);
	/*flags*/
	packet.push_back(static_cast<uint8_t>(flag));
	/*address*/
	packet.push_back(static_cast<uint8_t>(address));
	/*length*/
	packet.push_back(0);
	/*count*/
	packet.push_back(0);
	/*sum*/
	packet.push_back(0);
	for(uint8_t i = 2;i < (packet.size() - 1);++i)
		packet.back() ^= packet.at(i);

	servoWriteRead_.write(true);
	HAL_UART_Transmit(uart_ , packet.data(), static_cast<uint16_t>(packet.size()), 100);
	servoWriteRead_.write(false);
}


void RS40xCB::TorqueOn(std::vector<uint8_t>sendvector_short_torque_on){
	sendvector_short_torque_on.push_back(kData_TorqueOn);
	SendDataOfShortpacket(kHeader_ALLSERVO, memory_adr::TorqueOnOff, sendvector_short_torque_on);
	sendvector_short_torque_on.clear();
}

void RS40xCB::TorqueOff(std::vector<uint8_t>sendvector_short_torque_off){
	sendvector_short_torque_off.push_back(kData_torqueOff);
	SendDataOfShortpacket(kHeader_ALLSERVO, memory_adr::TorqueOnOff, sendvector_short_torque_off);
	sendvector_short_torque_off.clear();
}

void RS40xCB::write_rom(uint8_t id){
	SpecialTransmissionData(id , flags::WRITE_FLASH_ROM, memory_adr::SPECIAL_OPERATION);
}

void RS40xCB::reboot(uint8_t id){
	SpecialTransmissionData(id, flags::REBOOT, memory_adr::SPECIAL_OPERATION);
}

void RS40xCB::change_id(uint8_t old_id, uint8_t new_id, std::vector<uint8_t> sendvector_new_id){
	sendvector_new_id.push_back(new_id);
	SendDataOfShortpacket(old_id, memory_adr::ID, sendvector_new_id);
	sendvector_new_id.clear();
	HAL_Delay(300);
	write_rom(new_id);
	HAL_Delay(1200);
}


