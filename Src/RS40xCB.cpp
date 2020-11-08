
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


void RS40xCB::SendDataOfShortpacket(const uint8_t id, const uint8_t address, const std::vector<uint8_t>& data){
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
	packet.push_back(address);
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



void RS40xCB::TorqueOn(std::vector<uint8_t>sendector_short_torque_on){
	sendector_short_torque_on.push_back(kData_TorqueOn);
	SendDataOfShortpacket(kHeader_ALLSERVO, kAdr_TorqueOnOff, sendector_short_torque_on);
	sendector_short_torque_on.clear();
}

void RS40xCB::TorqueOff(std::vector<uint8_t>sendector_short_torque_off){
	sendector_short_torque_off.push_back(kData_torqueOff);
	SendDataOfShortpacket(kHeader_ALLSERVO, kAdr_TorqueOnOff, sendector_short_torque_off);
	sendector_short_torque_off.clear();
}

void RS40xCB::change_id(uint8_t old_id, char new_id){
	std::vector<uint8_t> packet;
	packet.reserve(8 + sizeof(char));

	/*header*/
	packet.push_back(kHeader_HighByte);
	packet.push_back(kHeader_LowByte);

	/*servo id*/
	packet.push_back(old_id);
	/*flags*/
	packet.push_back(0);
	/*address*/
	packet.push_back(kAdr_ID);
	/*length*/
	packet.push_back(sizeof(char));
	/*count*/
	packet.push_back(1);
	/*data*/
	packet.resize(7 + sizeof(char));
	memcpy(&packet[7], &new_id, sizeof(char));

	/*sum*/
	packet.push_back(0);
	for(uint8_t i = 2;i < (packet.size() - 1);++i)
		packet.back() ^= packet.at(i);

	servoWriteRead_.write(true);
	HAL_UART_Transmit(uart_, packet.data(), static_cast<uint16_t>(packet.size()), 100);
	servoWriteRead_.write(false);
}

void RS40xCB::write_rom(uint8_t id){
	std::vector<uint8_t> packet;
	packet.reserve(8);

	/*header*/
	packet.push_back(0xFA);
	packet.push_back(0xAF);

	/*servo id*/
	packet.push_back(id);
	/*flags*/
	packet.push_back(kFlag_WRITEFLASHROM);
	/*address*/
	packet.push_back(0xFF);
	/*length*/
	packet.push_back(0);
	/*count*/
	packet.push_back(0);
	/*sum*/
	packet.push_back(0);
	for(uint8_t i = 2;i < (packet.size() - 1);++i)
		packet.back() ^= packet.at(i);

	servoWriteRead_.write(true);
	HAL_UART_Transmit(uart_ , packet.data(), (uint16_t)packet.size(), 100);
	servoWriteRead_.write(false);
}
