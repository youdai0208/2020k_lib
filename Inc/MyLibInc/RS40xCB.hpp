/*
 * Futaba.hpp
 *
 *  Created on: 2019/07/28
 *      Author: tench
 */

#ifndef RS40XCB_HPP_
#define RS40XCB_HPP_

#include <vector>
#include <cstring>
#include"HAL_global.hpp"

class RS40xCB{
public:
	RS40xCB(const IOPin io);
	~RS40xCB();
	void Init(UART_HandleTypeDef* uarthandle);
	void send_data(uint8_t id, uint8_t adr, char data);
	void SendDataOfShortpacket(const uint8_t id, const uint8_t address, const std::vector<uint8_t>& data);
	void SendDataOfLongpacket(const std::vector<uint8_t> id,const uint8_t address, const std::vector<std::vector<uint8_t>> data);
	void TorqueOn(std::vector<uint8_t>sendector_short_torque_on);
	void TorqueOff(std::vector<uint8_t>sendector_short_torque_off);
	void change_id(uint8_t old_id, char new_id);
	void write_rom(uint8_t id);


	inline std::vector<uint8_t> get_shortpacketServoData(){return shortpacketServoData_;}
	inline std::vector<uint8_t> get_longpacketServoId(){return longpacketServoId_;}
	inline std::vector<std::vector<uint8_t>> get_longpacketServoData(){return longpacketServoData_;}
    inline void set_shortpacketServoData(uint8_t set_id){shortpacketServoData_.push_back(set_id);}
    inline void set_longpacketServoId(uint8_t set_id){longpacketServoId_.push_back(set_id);}
    inline void set_longpacketServoData(std::vector<uint8_t> set_data){longpacketServoData_.push_back(set_data);}
    inline void clear_longpacketServoId(){longpacketServoId_.clear();}
    inline void clear_longpacketServoData(){longpacketServoData_.clear();}

	//Header
	uint8_t kHeader_HighByte = 0xFA;
	uint8_t kHeader_LowByte = 0xAF;
	uint8_t kHeader_ALLSERVO = 0xFF;
	//Adress
	uint8_t kAdr_ID = 0x04;
	uint8_t kAdr_GOALPOSITION_L = 0x1E;
	uint8_t kAdr_TorqueOnOff = 0x24;
	//Flags
	uint8_t kFlag_WRITEFLASHROM = 0x40;
	uint8_t kFlag_REBOOT = 0x20;
	//Data
	uint8_t kData_TorqueOn = 0x01;
	uint8_t kData_torqueOff = 0x00;

private:
	std::vector<uint8_t> shortpacketServoData_;
	std::vector<uint8_t> longpacketServoId_;
	std::vector<std::vector<uint8_t>> longpacketServoData_;
	UART_HandleTypeDef* uart_;
	IOPin servoWriteRead_;
};


#endif /* RS40XCB_HPP_ */
