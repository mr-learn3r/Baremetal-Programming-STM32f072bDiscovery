/*
 * GPIO_drv.c
 *
 *  Created on: Apr 15, 2024
 *  Author: mr_learner
 *  Desc : This file contains info about function definition of GPIO_driver
 */

#include <GPIO_drv.h>
#include <stdio.h>


/* ===============================================================================================
 * Func name 			: GPIO_init
 *
 * Description			: This function initializes GPIO peripherals for a particular port
 *
 * Input parameter[0]	: Holds pointer to GPIO handle structure (contains base addr of port & pin config control)
 *
 * Return value			: NULL
 *
 * ===============================================================================================
 */

/* Initialize and Deinitialize */
GPIO_Status_t GPIO_init(GPIO_Handle_t *pGPIOhandle)
{

    uint32_t pinOffset = (2 * pGPIOhandle->pGPIO_Pinconfig->PinNum);

	/* GPIO pin mode configuration */
	if(pGPIOhandle->pGPIO_Pinconfig->PinMode<= GPIO_MODE_ANALOG)
	{
		/* Non interrupt mode */
		pGPIOhandle->pGPIOx->MODER  &= ~(0x3 << pinOffset); //Clearing pin
		pGPIOhandle->pGPIOx->MODER |= (pGPIOhandle->pGPIO_Pinconfig->PinMode << pinOffset);
	}
	else
	{
		/*Interrupt mode*/
		if(pGPIOhandle->pGPIO_Pinconfig->PinMode == GPIO_MODE_IT_FT)
		{
			/* Configure the FTSR */
			EXTI->FTSR |= (1 << pGPIOhandle->pGPIO_Pinconfig->PinNum);
			/* Clear corresponding RTSR bit */
			EXTI->RTSR &= ~(1 << pGPIOhandle->pGPIO_Pinconfig->PinNum);
		}
		else if(pGPIOhandle->pGPIO_Pinconfig->PinMode == GPIO_MODE_IT_RT)
		{
			/* Configure the RTSR */
			EXTI->RTSR |= (1 << pGPIOhandle->pGPIO_Pinconfig->PinNum);
			/* Clear corresponding FTSR bit */
			EXTI->FTSR &= ~(1 << pGPIOhandle->pGPIO_Pinconfig->PinNum);
		}
		else if(pGPIOhandle->pGPIO_Pinconfig->PinMode == GPIO_MODE_IT_FRT)
		{
			/* Configure the FTSR */
			EXTI->FTSR |= (1 << pGPIOhandle->pGPIO_Pinconfig->PinNum);
			/* Configure the RTSR */
			EXTI->RTSR |= (1 << pGPIOhandle->pGPIO_Pinconfig->PinNum);
		}
    }
    /* Configure GPIO port selection in SYSCFG_EXTICR */
     uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
     uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
     uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
     SYSCFG_PCLK_EN();
     SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

     /* Enable EXTI interrupt delivery using IMR */
     EXTI->IMR |= (1 << pGPIOhandle->pGPIO_Pinconfig->PinNum);

	        /* Configure GPIO port selection in SYSCFG_EXTICR */
	        uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
	        uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
	        uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
	        SYSCFG_PCLK_EN();
	        SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

	        /* Enable EXTI interrupt delivery using IMR */
	        EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	    }

    // GPIO pin mode configuration
    pGPIOhandle->pGPIOx->MODER &= ~(0x3 << pinOffset); // Clearing pin
    pGPIOhandle->pGPIOx->MODER |= (pGPIOhandle->pGPIO_Pinconfig->PinMode << pinOffset); // Setting pin

    // GPIO pin speed configuration
    pGPIOhandle->pGPIOx->OSPEEDR &= ~(0x3 << pinOffset); // Clearing pin
    pGPIOhandle->pGPIOx->OSPEEDR |= (pGPIOhandle->pGPIO_Pinconfig->PinSpeed << pinOffset); // Setting pin

    // GPIO pull up/down settings configuration
    pGPIOhandle->pGPIOx->PUPDR &= ~(0x3 << pinOffset); // Clearing pin
    pGPIOhandle->pGPIOx->PUPDR |= (pGPIOhandle->pGPIO_Pinconfig->PinPUPDControl << pinOffset); // Setting pin

	 if(pGPIOhandle->pGPIO_Pinconfig->PinMode == GPIO_MODE_ALTFN)
	    {
	        /* Alt functionality registers configuration */
	        uint8_t LowHigherRegGPIO;

	        LowHigherRegGPIO = pGPIOhandle->pGPIO_Pinconfig->PinNum / 8;
	        if(LowHigherRegGPIO>7)
			{
	        	pGPIOhandle->pGPIOx->AFRL &= ~(0xF << (4 * LowHigherRegGPIO));
	        	pGPIOhandle->pGPIOx->AFRL |= pGPIOhandle->pGPIO_Pinconfig->PinAltFuncMode << (4 * LowHigherRegGPIO);
			}
	        else
	        {
	        	pGPIOhandle->pGPIOx->AFRH &= ~(0xF << (4 * LowHigherRegGPIO));
	        	pGPIOhandle->pGPIOx->AFRH |= pGPIOhandle->pGPIO_Pinconfig->PinAltFuncMode << (4 * LowHigherRegGPIO);
	        }
	    }
	return GPIO_OK;
}

/* ===============================================================================================
 * Func name 			: GPIO_deinit
 *
 * Description			: This function de-initializes GPIO peripherals for a particular port
 *
 * Input parameter[0]	: Holds pointer to GPIO port base addr
 *
 * Return value			: NULL
 *
 * ===============================================================================================
 */
void GPIO_deinit(GPIO_Reg_t *pGPIOx)
{
		if(pGPIOx == GPIOA)
	    {
	        GPIOA_REG_RESET();
	    }
	    else if(pGPIOx == GPIOB)
	    {
	        GPIOB_REG_RESET();
	    }
	    else if(pGPIOx == GPIOC)
	    {
	        GPIOC_REG_RESET();
	    }
	    else if(pGPIOx == GPIOD)
	    {
	        GPIOD_REG_RESET();
	    }
	    else if(pGPIOx == GPIOE)
	    {
	        GPIOE_REG_RESET();
	    }
	    else if(pGPIOx == GPIOF)
	    {
	        GPIOF_REG_RESET();
	    }

}

/* Peripheral clock*/

/* ===============================================================================================
 * Func name 			: GPIO_pClckCtrl
 *
 * Description			: This function enable or disable peripheral clock  for a particular GPIO port
 *
 * Input parameter[0]	: Holds pointer to GPIO port base addr
 * Input parameter[1]	: Either ENABLE or DISABLE macro (0/1)
 *
 * Return value			: NULL
 *
 * ===============================================================================================
 */

GPIO_Status_t GPIO_pClckCtrl(GPIO_Reg_t *pGPIOx, uint8_t state) {
    if (state != ENABLE && state != DISABLE)
        return GPIO_ERROR_INVALID_STATE_VALUE;

    switch ((uint32_t)pGPIOx) {
        case (uint32_t)GPIOA:
            if (state == ENABLE) GPIOA_PCLCK_ENABLE();
            else GPIOA_PCLCK_DISABLE();
            break;
        case (uint32_t)GPIOB:
            if (state == ENABLE) GPIOB_PCLCK_ENABLE();
            else GPIOB_PCLCK_DISABLE();
            break;
        case (uint32_t)GPIOC:
            if (state == ENABLE) GPIOC_PCLCK_ENABLE();
            else GPIOC_PCLCK_DISABLE();
            break;
        case (uint32_t)GPIOD:
            if (state == ENABLE) GPIOD_PCLCK_ENABLE();
            else GPIOD_PCLCK_DISABLE();
            break;
        case (uint32_t)GPIOE:
            if (state == ENABLE) GPIOE_PCLCK_ENABLE();
            else GPIOE_PCLCK_DISABLE();
            break;
        case (uint32_t)GPIOF:
            if (state == ENABLE) GPIOF_PCLCK_ENABLE();
            else GPIOF_PCLCK_DISABLE();
            break;
        default:
            return GPIO_ERROR_INVALID_GPIO;
    }

    return GPIO_OK;
}


/*Read and write data operations*/
/* ===============================================================================================
 * Func name 			: GPIO_ReadFromInputPin
 *
 * Description			: This function reads from an input pin of a particular GPIO port
 *
 * Input parameter[0]	: Holds pointer to GPIO port base addr
 * Input parameter[1]	: Pin Number
 *
 * Return value			: Value ranging 0-255
 *
 * ===============================================================================================
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Reg_t *pGPIOx, uint8_t PinNum)
{
	uint8_t value;

	value = (uint8_t )((pGPIOx->IDR >> PinNum) & 0x00000001 );

	return value;

}

/* ===============================================================================================
 * Func name 			: GPIO_ReadFromInputPort
 *
 * Description			: This function reads from an input port of a particular GPIO port
 *
 * Input parameter[0]	: Holds pointer to GPIO port base addr
 *
 * Return value			: Value ranging [0,2^16-1]
 *
 * ===============================================================================================
 */
uint16_t GPIO_ReadFromInputPort			(GPIO_Reg_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t )pGPIOx->IDR;

	return value;
}

/* ===============================================================================================
 * Func name 			: GPIO_WriteToOutputPin
 *
 * Description			: This function writes to an output pin of a particular GPIO port
 *
 * Input parameter[0]	: Holds pointer to GPIO port base addr
 * Input parameter[1]	: Pin Number
 * Input parameter[2]	: Data ranging 0-255
 *
 * Return value			: Value ranging 0-255
 *
 * ===============================================================================================
 */
uint8_t GPIO_WriteToOutputPin (GPIO_Reg_t *pGPIOx, uint8_t PinNum, uint8_t data)
{
	  if(data == SET)
	    {
	        /* Write 1 to the output data register at the bit field corresponding to the pin number */
	        pGPIOx->ODR |= ( 1 << PinNum );
	    }
	    else
	    {
	        /* Write 0 to the output data register at the bit field corresponding to the pin number */
	        pGPIOx->ODR &= ~( 1 << PinNum );	//Clear pin
	    }
  return data;
}

/* ===============================================================================================
 * Func name 			: GPIO_WriteToOutputPort
 *
 * Description			: This function writes to an output port of a particular GPIO port
 *
 * Input parameter[0]	: Holds pointer to GPIO port base addr
 * Input parameter[2]	: Data ranging [0,2^16-1]
 *
 * Return value			: Value ranging 0-255
 *
 * ===============================================================================================
 */
uint8_t GPIO_WriteToOutputPort	(GPIO_Reg_t *pGPIOx, uint16_t data)
{
	pGPIOx->ODR = data;

	return data?1:0;
}

/* ===============================================================================================
 * Func name 			: GPIO_ToggleOutputPin
 *
 * Description			: This function toggles an output pin of a particular GPIO port
 *
 * Input parameter[0]	: Holds pointer to GPIO port base addr
 * Input parameter[1]	: Pin Number
 *
 * Return value			: NULL
 *
 * ===============================================================================================
 */
void GPIO_ToggleOutputPin  				(GPIO_Reg_t *pGPIOx, uint8_t PinNum)
{

	pGPIOx->ODR ^= (1 << PinNum);

}

/*Interrupt config and handling*/

/* ===============================================================================================
 * Func name 			: GPIO_IRQConfig
 *
 * Description			: This function configures interrupt for GPIO
 *
 * Input parameter[0]	: IRQ number
 * Input parameter[1]	: ENABLE or DISABLE macro (0/1)
 *
 * Return value			: NULL
 *
 * ===============================================================================================
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t status)
{


}

/* ===============================================================================================
 * Func name 			: GPIO_IRQPriorityConfig
 *
 * Description			: This function configures interrupt for GPIO
 *
 * Input parameter[0]	: IRQ number
 * Input parameter[1]	: IRQ priority
 *
 * Return value			: NULL
 *
 * ===============================================================================================
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{


}
/* ===============================================================================================
 * Func name 			: GPIO_IRQHandler
 *
 * Description			: This function handles GPIO interrupts
 *
 * Input parameter[0]	: Pin number
 *
 * Return value			: NULL
 *
 * ===============================================================================================
 */
void GPIO_IRQHandler(uint8_t PinNum)
{


}


