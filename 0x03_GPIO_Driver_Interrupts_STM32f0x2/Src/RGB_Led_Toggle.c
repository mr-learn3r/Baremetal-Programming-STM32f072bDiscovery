/*
 * RGB_Led_Toggle.c
 *
 *  Created on: Apr 16, 2024
 *  Author: mr_learner
 *  Desc : This file contains user space code for toggling RGB light using GPIO_driver
 */

#include <STM32f072b.h>
#include <GPIO_drv.h>
#include <stdlib.h>

/*Delay function*/
void delay (void)
{
	for(uint32_t x=0;x<500000;++x);
}

int main(void)
{
	GPIO_Handle_t Red_Led;
	GPIO_Handle_t Green_Led;
	GPIO_Handle_t Blue_Led;

	Red_Led.pGPIOx = GPIOA;
    Red_Led.pGPIO_Pinconfig = &(GPIO_Pinconfig_t) {
        .PinNum = GPIO_PIN_NO_8,
        .PinMode = GPIO_MODE_OUT,
        .PinSpeed = GPIO_SPEED_FAST,
        .PinPUPDControl = GPIO_PIN_NO_PUPD,
        .PinOPType = GPIO_OP_TYPE_PP
    };

    Green_Led.pGPIOx = GPIOA;
    Green_Led.pGPIO_Pinconfig = &(GPIO_Pinconfig_t) {
        .PinNum = GPIO_PIN_NO_9,
        .PinMode = GPIO_MODE_OUT,
        .PinSpeed = GPIO_SPEED_FAST,
        .PinPUPDControl = GPIO_PIN_NO_PUPD,
        .PinOPType = GPIO_OP_TYPE_PP
    };

    Blue_Led.pGPIOx = GPIOA;
    Blue_Led.pGPIO_Pinconfig = &(GPIO_Pinconfig_t) {
        .PinNum = GPIO_PIN_NO_10,
        .PinMode = GPIO_MODE_OUT,
        .PinSpeed = GPIO_SPEED_FAST,
        .PinPUPDControl = GPIO_PIN_NO_PUPD,
        .PinOPType = GPIO_OP_TYPE_PP
    };

    if (GPIO_pClckCtrl(Red_Led.pGPIOx, ENABLE) != GPIO_OK ||
           GPIO_init(&Red_Led) != GPIO_OK ||
           GPIO_init(&Green_Led) != GPIO_OK ||
           GPIO_init(&Blue_Led) != GPIO_OK) {
           // Handle error
           return -1;
    }

	while(1)
	{

	 GPIO_WriteToOutputPin (Green_Led.pGPIOx, Green_Led.pGPIO_Pinconfig->PinNum, 0);
	 GPIO_WriteToOutputPin (Blue_Led.pGPIOx, Blue_Led.pGPIO_Pinconfig->PinNum, 0);
	 GPIO_ToggleOutputPin(Red_Led.pGPIOx, Red_Led.pGPIO_Pinconfig->PinNum);
     delay();

     GPIO_WriteToOutputPin (Red_Led.pGPIOx, Red_Led.pGPIO_Pinconfig->PinNum, 0);
     GPIO_WriteToOutputPin (Blue_Led.pGPIOx, Blue_Led.pGPIO_Pinconfig->PinNum, 0);
	 GPIO_ToggleOutputPin(Green_Led.pGPIOx, Green_Led.pGPIO_Pinconfig->PinNum);
	 delay();

	 GPIO_WriteToOutputPin (Red_Led.pGPIOx, Red_Led.pGPIO_Pinconfig->PinNum, 0);
	 GPIO_WriteToOutputPin (Green_Led.pGPIOx, Green_Led.pGPIO_Pinconfig->PinNum, 0);
	 GPIO_ToggleOutputPin(Blue_Led.pGPIOx, Blue_Led.pGPIO_Pinconfig->PinNum);
	 delay();
	}

	GPIO_deinit(Red_Led.pGPIOx);
	GPIO_deinit(Green_Led.pGPIOx);
	GPIO_deinit(Blue_Led.pGPIOx);

return 0;
}


