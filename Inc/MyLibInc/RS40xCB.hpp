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

    enum class memory_adr :const uint8_t{
		ID = 0x04,
		BAUD_RATE = 0x06,
		RETURN_DELAY,
		CW_ANGLE_LIMIT_L,
		CW_ANGLE_LIMIT_H,
		CCW_ANGLE_LIMIT_L,
		CCW_ANGLE_LIMIT_H,
		TEMPERATURE_LIMIT_L = 0x0E,
		TEMPERATURE_LIMIT_H,
		DAMPER = 0x14,
		TORQUE_IN_SILENCE = 0x16,
		WARM_UP_TIME,
		CW_COMPLIANCE_MARGIN,
		CCW_COMPLIANCE_MARGIN,
		CW_COMPLIANCE_SLOPE,
		CCW_COMPLIANCE_SLOPE,
		PUNCH_L,
		PUNCH_H,
		/*RAM*/
		GOAL_POSITION_L,
		GOAL_POSITION_H,
		GOAL_TIME_L,
		GOAL_TIME_H,
		MAX_TORQUE = 0x23,
		TorqueOnOff,
		PRESENT_POSITION_L = 0x2A,
		PRESENT_POSITION_H,
		PRESENT_TIME_L,
		PRESENT_TIME_H,
		PRESENT_SPEED_L,
		PRESENT_SPEED_H,
		PRESENT_CURRENT_L,
		PRESENT_CURRENT_H,
		PRESENT_TEMPERATURE_L,
		PRESENT_TEMPERATURE_H,
		PRESENT_VOLTS_L,
		PRESENT_VOLTS_H,
		SPECIAL_OPERATION = 0xFF
    };

    enum class flags : uint8_t{
		RETURN_ADR_1 = 0x01,
		RETURN_ADR_2 = 0x02,
		RETURN_ADR_3 = 0x04,
		RETURN_ADR_4 = 0x08,
		RETURN_ADR_ALL = 0x0F,
		MEMORY_INITIALIZE = 0x10,
		REBOOT = 0x20,
		WRITE_FLASH_ROM = 0x40
	};
    enum class return_flags : uint8_t{
		READ_PACET_ERROR = 0x02,
		WRITE_ROM_ERROR = 0x08,
		TEMPERATURE_LIMIT_ARRAM = 0x20,
		TEMPERATURE_LIMIT_ERROR = 0x80
	};

	RS40xCB(const IOPin io);
	~RS40xCB();
	void Init(UART_HandleTypeDef* uarthandle);
	void SendDataOfShortpacket(uint8_t id, memory_adr address, std::vector<uint8_t> data);
	void SendDataOfLongpacket(std::vector<uint8_t> id, uint8_t address, std::vector<std::vector<uint8_t>> data);
	void SpecialTransmissionData(uint8_t id, flags flag, memory_adr address);
	void TorqueOn(std::vector<uint8_t>sendector_short_torque_on);
	void TorqueOff(std::vector<uint8_t>sendector_short_torque_off);
	void write_rom(uint8_t id);
	void reboot(uint8_t id);
	void change_id(uint8_t old_id, uint8_t new_id, std::vector<uint8_t> sendvector_new_id);


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
	uint8_t kHeader_LowByte  = 0xAF;
	uint8_t kHeader_ALLSERVO = 0xFF;
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
