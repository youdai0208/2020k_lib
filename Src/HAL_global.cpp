#include <MyLibInc/RS40xCB.hpp>

void IOPin::write(bool state){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING_FALLING;
    if(state == PULLUP){
        GPIO_InitStruct.Pull = GPIO_PULLUP;
    }
    else if(state == PULLDOWN){
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    }
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}
