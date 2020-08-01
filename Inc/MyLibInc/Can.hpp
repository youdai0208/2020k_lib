/*
 * CAN.hpp
 *
 *  Created on: 2019/06/29
 *      Author: youda
 */

#ifndef MYLIBINC_CAN_HPP_
#define MYLIBINC_CAN_HPP_

#include <array>
#include "stm32f3xx_hal.h"

class Can{
public:
	Can(/*const CAN_HandleTypeDef &hcan*/);
	~Can();
	void setFilter(const uint8_t set_address);
	void sendRemote(const uint8_t address);
	template<std::size_t N>
	bool send(const uint8_t address, uint8_t (&send_data)[N]){
		CAN_TxHeaderTypeDef canTxHeaderStruct;
		uint32_t mail_box;
		canTxHeaderStruct.StdId 				= address;
		canTxHeaderStruct.DLC 					= N;
		canTxHeaderStruct.ExtId 				= 0x00;
		canTxHeaderStruct.RTR 					= CAN_RTR_DATA;
		canTxHeaderStruct.IDE 					= CAN_ID_STD;
		canTxHeaderStruct.TransmitGlobalTime 	= DISABLE;

		while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan_));

		if(HAL_CAN_AddTxMessage(&hcan_, &canTxHeaderStruct, send_data, &mail_box) != HAL_OK){
			HAL_CAN_AbortTxRequest(&hcan_, mail_box);
			if(HAL_CAN_AddTxMessage(&hcan_, &canTxHeaderStruct, send_data, &mail_box) != HAL_OK){
	//			Error_Handler();
				return false;
			}
		}

		return true;
	}

	template<std::size_t N>
	bool send(const uint8_t address, const std::array<uint8_t, N> &send_data){
		CAN_TxHeaderTypeDef canTxHeaderStruct;
		uint32_t mail_box;
		uint8_t copy_send_data[N] = {0};
		canTxHeaderStruct.StdId 				= address;
		canTxHeaderStruct.DLC 					= N;
		canTxHeaderStruct.ExtId 				= 0x00;
		canTxHeaderStruct.RTR 					= CAN_RTR_DATA;
		canTxHeaderStruct.IDE 					= CAN_ID_STD;
		canTxHeaderStruct.TransmitGlobalTime 	= DISABLE;

		for(auto copy_send_data:send_data);

		while(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan_));

		if(HAL_CAN_AddTxMessage(&hcan_, &canTxHeaderStruct, copy_send_data, &mail_box) != HAL_OK){
			HAL_CAN_AbortTxRequest(&hcan_, mail_box);
			if(HAL_CAN_AddTxMessage(&hcan_, &canTxHeaderStruct, copy_send_data, &mail_box) != HAL_OK){
	//			Error_Handler();
				return false;
			}
		}

		return true;
	}

//	bool send(const uint8_t address, uint8_t (&send_data)[1]);
//	bool send(const uint8_t address, uint8_t (&send_data)[5]);

	inline void setUp(const CAN_HandleTypeDef &hcan, const uint8_t set_address, const uint32_t receive_interrupt){
//		hcan_ = hcan;
		receive_interrupt_ = receive_interrupt;
		setCANHandle(hcan);
		setFilter(set_address);
		HAL_CAN_ActivateNotification(&hcan_, receive_interrupt_);
		HAL_CAN_Start(&hcan_);
	}
	inline void setCANHandle(const CAN_HandleTypeDef &hcan){
		hcan_ = hcan;
//		HAL_CAN_Start(&hcan_);
	}
	inline std::array<uint8_t, 8> getMessage(const uint8_t can_rx_fifo){
		std::array<uint8_t, 8> result;
		CAN_RxHeaderTypeDef RxHeader;
		uint8_t rx_data[8] = {0};

		HAL_CAN_GetRxMessage(&hcan_, can_rx_fifo, &RxHeader, rx_data);

		for(uint8_t i = 0;i < sizeof(rx_data); i++){
			result.at(i) = rx_data[i];
		}

		return result;
	}

private:
	CAN_HandleTypeDef hcan_;
	uint32_t receive_interrupt_ = CAN_IT_RX_FIFO0_MSG_PENDING;
};



#endif /* MYLIBINC_CAN_HPP_ */
