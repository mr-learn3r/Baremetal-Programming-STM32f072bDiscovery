/*
 * Red_Led_Toggle_Button_Interrupt.c
 *
 *  Created on: Apr 29, 2024
 *  Author: mr_learner
 *  Desc : This file contains user space code for toggling Red LED light using Button Interrupt
 */

#include <string.h>
#include <STM32f072b.h>
#include <GPIO_drv.h>

#define DELAY  100000

void delay(void)
{
    for(uint32_t i = 0; i < DELAY; i++);
}

int main(void)
{
    GPIO_Handle_t Red_Led, Button;

    memset(&Red_Led, 0, sizeof(Red_Led));
    memset(&Button, 0, sizeof(Button));

	Red_Led.pGPIOx = GPIOA;
    Red_Led.pGPIO_Pinconfig = &(GPIO_Pinconfig_t) {
        .PinNum = GPIO_PIN_NO_8,
        .PinMode = GPIO_MODE_OUT,
        .PinSpeed = GPIO_SPEED_FAST,
        .PinPUPDControl = GPIO_PIN_NO_PUPD,
        .PinOPType = GPIO_OP_TYPE_PP
    };

    Button.pGPIOx = GPIOD;
    Button.pGPIO_Pinconfig = &(GPIO_Pinconfig_t) {
        .PinNum = GPIO_PIN_NO_2,
        .PinMode = GPIO_MODE_IT_FT,
        .PinSpeed = GPIO_SPEED_FAST,
        .PinPUPDControl = GPIO_PIN_PU
    };

    if (GPIO_pClckCtrl(Red_Led.pGPIOx, ENABLE) != GPIO_OK ||
    	GPIO_pClckCtrl(Button.pGPIOx, ENABLE) != GPIO_OK ||
           GPIO_init(&Red_Led) != GPIO_OK ||
           GPIO_init(&Button) != GPIO_OK) {
           // Handle error
           return -1;
    }


    GPIO_WriteToOutputPin(GPIOA,Red_Led.pGPIO_Pinconfig->PinNum,0);

    /* IRQ configuration */
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI2_3, NVIC_IRQ_PRI3);
    GPIO_IRQConfig(IRQ_NO_EXTI2_3, ENABLE);

    while(1);

    return 0;
}

void EXTI2_3_IRQHandler(void)
{
	delay();
    GPIO_IRQHandler(GPIO_PIN_NO_2);
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
}
