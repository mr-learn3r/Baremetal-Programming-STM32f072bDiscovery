/*
 * STM32f072b.h
 *
 *  Created on: Apr 15, 2024
 *  Author: mr_learner
 *  Desc : This file contains info about bus and peripherals specific to micro-controller on board
 */

#ifndef INC_STM32F072B_H_
#define INC_STM32F072B_H_

#include <stdint.h>

/*
 *  Base address for Flash and SRAM memory
*/

#define FLASH_BASEADDR      	0x08000000U
#define SRAM_BASEADDR			0x20000000U

/*
 *  Base address for AHBx & APB peripheral Bus
*/

#define APBPERIPH_BASEADDR  	0x40000000U
#define AHB1PERIPH_BASEADDR  	0x40020000U
#define AHB2PERIPH_BASEADDR  	0x48000000U


/*
 * Base address of GPIO peripherals on AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB2PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB2PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB2PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB2PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB2PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB2PERIPH_BASEADDR + 0x1400)


/*
 * Base address of RCC on AHB1 bus
 */
#define RCC_BASEADDR            (AHB1PERIPH_BASEADDR + 0x1000)

/*
 * Base addresses of peripherals on APB bus
 */

#define EXTI_BASEADDR			(APBPERIPH_BASEADDR  + 0x0400)  /*EXTI Base address*/
#define SYSCFG_BASEADDR			(APBPERIPH_BASEADDR  + 0x0000)  /*SYSCFG Base address*/

/*============================================================================================================*/


/* 								Peripheral Registers structure												  */


typedef struct{
    volatile uint32_t MODER;    /*	GPIO port mode register										Addr Offset: 0x00*/
    volatile uint32_t OTYPER;   /*	GPIO port output type register								Addr Offset: 0x04*/
    volatile uint32_t OSPEEDR;  /*	GPIO port output speed register								Addr Offset: 0x08*/
    volatile uint32_t PUPDR;    /*	GPIO port pull-up/pull-down register						Addr Offset: 0x0C*/
    volatile uint32_t IDR;      /*	GPIO port input data register								Addr Offset: 0x10*/
    volatile uint32_t ODR;      /*	GPIO port output data register								Addr Offset: 0x14*/
    volatile uint32_t BSSR;     /*	GPIO port bit set/reset register							Addr Offset: 0x18*/
    volatile uint32_t LCKR;     /*	GPIO port configuration lock register						Addr Offset: 0x1C*/
    volatile uint32_t AFRL;     /*	GPIO alternate function low register						Addr Offset: 0x20*/
    volatile uint32_t AFRH;     /*	GPIO alternate function high register						Addr Offset: 0x24*/
    volatile uint32_t BRR;      /*	GPIO port bit reset register								Addr Offset: 0x28*/
}GPIO_Reg_t;


/* 								RCC Registers structure												  */
typedef struct{
    volatile uint32_t RCC_CR;    	/*	Clock control register										Addr Offset: 0x00*/
    volatile uint32_t RCC_CFGR;  	/*	Clock configuration register								Addr Offset: 0x04*/
    volatile uint32_t RCC_CIR;   	/*	Clock interrupt register									Addr Offset: 0x08*/
    volatile uint32_t RCC_APB2RSTR; /*	APB peripheral reset register 2								Addr Offset: 0x0C*/
    volatile uint32_t RCC_APB1RSTR; /*	APB peripheral reset register 1								Addr Offset: 0x10*/
    volatile uint32_t RCC_AHBENR;   /*	AHB peripheral clock enable register						Addr Offset: 0x14*/
    volatile uint32_t RCC_APB2ENR;  /*	APB peripheral clock enable register 2						Addr Offset: 0x18*/
    volatile uint32_t RCC_APB1ENR;  /*	APB peripheral clock enable register 1						Addr Offset: 0x1C*/
    volatile uint32_t RCC_BDCR;     /*	RTC domain control register									Addr Offset: 0x20*/
    volatile uint32_t RCC_CSR;      /*	Control/status register										Addr Offset: 0x24*/
    volatile uint32_t RCC_AHBRSTR;  /*	AHB peripheral reset register								Addr Offset: 0x28*/
    volatile uint32_t RCC_CFGR2;    /*	Clock configuration register 2								Addr Offset: 0x2C*/
    volatile uint32_t RCC_CFGR3;    /*	Clock configuration register 3								Addr Offset: 0x30*/
    volatile uint32_t RCC_CR2;      /*	Clock control register 2									Addr Offset: 0x34*/
}RCC_Reg_t;


/* 								EXTI Registers structure												  */
typedef struct{
    volatile uint32_t IMR;      /* EXTI Interrupt mask register,            Address offset: 0x00 */
    volatile uint32_t EMR;      /* EXTI Event mask register,                Address offset: 0x04 */
    volatile uint32_t RTSR;     /* EXTI Rising trigger selection register,  Address offset: 0x08 */
    volatile uint32_t FTSR;     /* EXTI Falling trigger selection register, Address offset: 0x0C */
    volatile uint32_t SWIER;    /* EXTI Software interrupt event register,  Address offset: 0x10 */
    volatile uint32_t PR;       /* EXTI Pending register,                   Address offset: 0x14 */
}EXTI_Reg_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
    volatile uint32_t SYSCFG_CFGR1;       /*                         Address offset: 0x00    */
    volatile uint32_t SYSCFG_EXTICR[4];   /* SYSCFG External interrupt configuration register,    Address offset: 0x08-14 */
    volatile uint32_t SYSCFG_CFGR2;       /*                         Address offset: 0x18    */
    uint32_t      reserved[24]; /* SYSCFG Reserved register,                            Address offset: 0x1D-7F */
    uint16_t      dummy;
    volatile uint32_t SYSCFG_ITLINE[30];  /* SYSCFG Configuration register,                       Address offset: 0x80-0xF8*/
}SYSCFG_Reg_t;

/*					GPIO peripheral base addr typecast with register structure ptr							  */

#define GPIOA   ((GPIO_Reg_t*)GPIOA_BASEADDR)
#define GPIOB   ((GPIO_Reg_t*)GPIOB_BASEADDR)
#define GPIOC   ((GPIO_Reg_t*)GPIOC_BASEADDR)
#define GPIOD   ((GPIO_Reg_t*)GPIOD_BASEADDR)
#define GPIOE   ((GPIO_Reg_t*)GPIOE_BASEADDR)
#define GPIOF   ((GPIO_Reg_t*)GPIOF_BASEADDR)


/*					RCC base addr typecast with register structure ptr							  			  */

#define RCC     ((RCC_Reg_t*)RCC_BASEADDR)

/*					EXTI base addr typecast with register structure ptr							  			  */
#define EXTI     ((EXTI_Reg_t*)EXTI_BASEADDR)

/*================================================================================================================*/

/*Clock enable & disable macros for GPIOx ports */

#define GPIOA_PCLCK_ENABLE()	(RCC->RCC_AHBENR |= (1<<17))
#define GPIOB_PCLCK_ENABLE()	(RCC->RCC_AHBENR |= (1<<18))
#define GPIOC_PCLCK_ENABLE()	(RCC->RCC_AHBENR |= (1<<19))
#define GPIOD_PCLCK_ENABLE()	(RCC->RCC_AHBENR |= (1<<20))
#define GPIOE_PCLCK_ENABLE()	(RCC->RCC_AHBENR |= (1<<21))
#define GPIOF_PCLCK_ENABLE()	(RCC->RCC_AHBENR |= (1<<22))


/* Clock enable Macros for SYSCFG peripheral   */
#define SYSCFG_PCLCK_ENABLE()    ( RCC->RCC_APB2ENR |= 1 ) /* SYSCFG peripheral clock enabled */


#define GPIOA_PCLCK_DISABLE()	(RCC->RCC_AHBENR &= ~(1<<17))
#define GPIOB_PCLCK_DISABLE()	(RCC->RCC_AHBENR &= ~(1<<18))
#define GPIOC_PCLCK_DISABLE()	(RCC->RCC_AHBENR &= ~(1<<19))
#define GPIOD_PCLCK_DISABLE()	(RCC->RCC_AHBENR &= ~(1<<20))
#define GPIOE_PCLCK_DISABLE()	(RCC->RCC_AHBENR &= ~(1<<21))
#define GPIOF_PCLCK_DISABLE()	(RCC->RCC_AHBENR &= ~(1<<22))


/* Macros to reset GPIOx peripherals */
#define GPIOA_REG_RESET()   do{ (RCC->RCC_AHBRSTR |= (1 << 17)); (RCC->RCC_AHBRSTR &= ~(1 << 17)); }while(0)
#define GPIOB_REG_RESET()   do{ (RCC->RCC_AHBRSTR |= (1 << 18)); (RCC->RCC_AHBRSTR &= ~(1 << 18)); }while(0)
#define GPIOC_REG_RESET()   do{ (RCC->RCC_AHBRSTR |= (1 << 19)); (RCC->RCC_AHBRSTR &= ~(1 << 19)); }while(0)
#define GPIOD_REG_RESET()   do{ (RCC->RCC_AHBRSTR |= (1 << 20)); (RCC->RCC_AHBRSTR &= ~(1 << 20)); }while(0)
#define GPIOE_REG_RESET()   do{ (RCC->RCC_AHBRSTR |= (1 << 21)); (RCC->RCC_AHBRSTR &= ~(1 << 21)); }while(0)
#define GPIOF_REG_RESET()   do{ (RCC->RCC_AHBRSTR |= (1 << 22)); (RCC->RCC_AHBRSTR &= ~(1 << 22)); }while(0)

/*================================================================================================================*/

/* Macros for states*/

#define ENABLE 				0
#define DISABLE				1
#define	SET					ENABLE
#define RESET				DISABLE


#endif /* INC_STM32F072B_H_ */
