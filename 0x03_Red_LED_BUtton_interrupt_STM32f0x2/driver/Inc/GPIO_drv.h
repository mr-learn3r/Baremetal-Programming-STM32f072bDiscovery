/*
 * GPIO_drv.h
 *
 *  Created on: Apr 15, 2024
 *  Author: mr_learner
 *  Desc : This file contains info about function declaration and structs used in GPIO driver
 */

#ifndef INC_GPIO_DRV_H_
#define INC_GPIO_DRV_H_

#include "STM32f072b.h"

typedef struct{
	uint8_t PinNum;                 /*Values from @GPIO_PIN_NUMBERS*/
	uint8_t PinMode;				/*Values from @GPIO_PIN_MODES*/
	uint8_t PinSpeed;				/*Values from @GPIO_PIN_SPEED*/
	uint8_t PinPUPDControl;			/*Values from @GPIO_PIN_PULL_UP_PULL_DOWN*/
	uint8_t PinOPType;				/*Values from @GPIO_PIN_OUTPUT_TYPE*/
	uint8_t PinAltFuncMode;			/*Values only when @GPIO_PIN_MODES set to GPIO_MODE_ALTFN (2)*/
}GPIO_Pinconfig_t;


/* GPIO handle structure*/

typedef struct{
	GPIO_Reg_t *pGPIOx;                 /*	Stores GPIO port base address	*/
	GPIO_Pinconfig_t *pGPIO_Pinconfig;  /*	Stores GPIO pin configuration setting	*/
}GPIO_Handle_t;

/*Enum for storing the states of GPIO functions for error checking*/
typedef enum {
    GPIO_OK = 0,
    GPIO_ERROR_INVALID_STATE,
    GPIO_ERROR_INVALID_GPIO,
    GPIO_ERROR_INVALID_STATE_VALUE
} GPIO_Status_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0       0
#define GPIO_PIN_NO_1       1
#define GPIO_PIN_NO_2       2
#define GPIO_PIN_NO_3       3
#define GPIO_PIN_NO_4       4
#define GPIO_PIN_NO_5       5
#define GPIO_PIN_NO_6       6
#define GPIO_PIN_NO_7       7
#define GPIO_PIN_NO_8       8
#define GPIO_PIN_NO_9       9
#define GPIO_PIN_NO_10      10
#define GPIO_PIN_NO_11      11
#define GPIO_PIN_NO_12      12
#define GPIO_PIN_NO_13      13
#define GPIO_PIN_NO_14      14
#define GPIO_PIN_NO_15      15


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN        0 /* GPIO Input mode                          */
#define GPIO_MODE_OUT       1 /* GPIO Output mode                         */
#define GPIO_MODE_ALTFN     2 /* GPIO Alternate functionality             */
#define GPIO_MODE_ANALOG    3 /* GPIO Analog mode                         */
#define GPIO_MODE_IT_FT     4 /* GPIO Input mode falling edge trigger     */
#define GPIO_MODE_IT_RT     5 /* GPIO Input mode rising edge trigger      */
#define GPIO_MODE_IT_FRT    6 /* GPIO Input mode fall-rising edge trigger */


/*
 * @GPIO_PIN_OUTPUT_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP     0 /* GPIO Output type push-pull mode  */
#define GPIO_OP_TYPE_OD     1 /* GPIO Output type open-drain mode */


/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST     2
#define GPIO_SPEED_HIGH     3


/*
 * @GPIO_PIN_PULL_UP_PULL_DOWN
 * GPIO pin pull-up and pull-down configuration macros
 */
#define GPIO_PIN_NO_PUPD    0 /* GPIO configuration no pull-up, pull-down */
#define GPIO_PIN_PU         1 /* GPIO configuration pull-up               */
#define GPIO_PIN_PD         2 /* GPIO configuration pull-down             */


/*=======================================================================================*/

/*								Function declaration of Driver APIs										*/

/* Initialize and Deinitialize */
GPIO_Status_t GPIO_init							(GPIO_Handle_t *pGPIOhandle);
void GPIO_deinit						(GPIO_Reg_t *pGPIOx);

/* Peripheral clock*/
GPIO_Status_t GPIO_pClckCtrl						(GPIO_Reg_t *pGPIOx, uint8_t state);

/*Read and write data operations*/
uint8_t GPIO_ReadFromInputPin 			(GPIO_Reg_t *pGPIOx, uint8_t PinNum);
uint16_t GPIO_ReadFromInputPort			(GPIO_Reg_t *pGPIOx);
uint8_t GPIO_WriteToOutputPin 			(GPIO_Reg_t *pGPIOx, uint8_t PinNum, uint8_t data);
uint8_t GPIO_WriteToOutputPort			(GPIO_Reg_t *pGPIOx, uint16_t data);
void GPIO_ToggleOutputPin  				(GPIO_Reg_t *pGPIOx, uint8_t PinNum);

/*Interrupt config and handling*/
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t status);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandler(uint8_t PinNum);

#endif /* INC_GPIO_DRV_H_ */
