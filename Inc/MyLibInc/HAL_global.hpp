/*
 * HAL_global.hpp
 *
 *  Created on: 2019/07/28
 *      Author: tench
 */

#ifndef HAL_GLOBAL_HPP_
#define HAL_GLOBAL_HPP_

/*
 * Change according to the board you use
 */
extern "C"{
#ifdef STM32F405xx
#include "stm32f4xx_hal.h"
#endif
}

class IOPin{
public:
	IOPin(GPIO_TypeDef* io_port, uint16_t io_pin):GPIOx(io_port), GPIO_Pin(io_pin){};
	void write(bool state);

	~IOPin(){};
private:
	GPIO_TypeDef* const	GPIOx;
	const uint16_t		GPIO_Pin;

	static constexpr bool PULLUP = true;
	static constexpr bool PULLDOWN = false;
};

#endif /* HAL_GLOBAL_HPP_ */
