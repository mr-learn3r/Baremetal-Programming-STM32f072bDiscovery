/*
 * STM32f072b.h
 *
 *  Created on: Apr 15, 2024
 *  Author: mr_learner
 *  Desc : This file contains info about bus and peripherals specific to micro-controller on board & processor specific for NVIC
 */

#ifndef INC_STM32F072B_H_
#define INC_STM32F072B_H_

#include <stdint.h>

/**********************************ARM Cortex M0 Processor Specific Details **********************************/
/* NVIC ISERx Register Addresses*/
#define NVIC_ISER      ( (volatile uint32_t*)0xE000E100 )
#define NVIC_ICER     ( (volatile uint32_t*)0xE000E180 )

/* NVIC Priority Register Base Address */
#define NVIC_PR_BASE_ADDR   ( (volatile uint32_t*)0XE000E400 )


/*Number of priority bits implemented in Priority Register */
#define NO_PR_BITS_IMPLEMENTED  2
/*
 *  Base address for Flash and SRAM memory
*/

#define FLASH_BASEADDR      	0x08000000U
#define SRAM_BASEADDR			0x20000000U

/*
 *  Base address for AHBx & APB peripheral Bus
*/

#define APB1PERIPH_BASEADDR  	0x40000000U
#define APB2PERIPH_BASEADDR		0x40010000U
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

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR  + 0x0400)  /*EXTI Base address*/
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR  + 0x0000)  /*SYSCFG Base address*/

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
    volatile uint32_t EXTI_IMR;      /* EXTI Interrupt mask register,            Address offset: 0x00 */
    volatile uint32_t EXTI_EMR;      /* EXTI Event mask register,                Address offset: 0x04 */
    volatile uint32_t EXTI_RTSR;     /* EXTI Rising trigger selection register,  Address offset: 0x08 */
    volatile uint32_t EXTI_FTSR;     /* EXTI Falling trigger selection register, Address offset: 0x0C */
    volatile uint32_t EXTI_SWIER;    /* EXTI Software interrupt event register,  Address offset: 0x10 */
    volatile uint32_t EXTI_PR;       /* EXTI Pending register,                   Address offset: 0x14 */
}EXTI_Reg_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
    volatile uint32_t SYSCFG_CFGR1;       /*                         Address offset: 0x00    */
    uint32_t dummy0;
    volatile uint32_t SYSCFG_EXTICR[4];   /* SYSCFG External interrupt configuration register,    Address offset: 0x08-14 */
    volatile uint32_t SYSCFG_CFGR2;       /*                         Address offset: 0x18    */
    uint32_t dummy1;
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

#define RCC      ((RCC_Reg_t*)RCC_BASEADDR)

/*					EXTI base addr typecast with register structure ptr							  			  */
#define EXTI     ((EXTI_Reg_t*)EXTI_BASEADDR)

/*					SYSCFG base addr typecast with register structure ptr				    	  			  */
#define SYSCFG   ((SYSCFG_Reg_t*) SYSCFG_BASEADDR)
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

/*IRQ(Interrupt Request) Numbers of STM32F0x2 MCU*/

#define IRQ_NO_WWDG					0 		/* Window Watchdog interrupt                             */
#define IRQ_NO_PVD					1 		/* PVD and VDDIO2 supply comparator interrupt            */
#define IRQ_NO_RTC					2 		/* RTC interrupts                                        */
#define IRQ_NO_FLASH             	3		/* Flash global interrupt                                */
#define IRQ_NO_RCC_CRS           	4		/* RCC and CRS global interrupts                         */
#define IRQ_NO_EXTI0_1            	5		/* EXTI Line[1:0] interrupts                             */
#define IRQ_NO_EXTI2_3            	6		/* EXTI Line[3:2] interrupts                             */
#define IRQ_NO_EXTI4_15          	7		/* EXTI Line15 and EXTI4 interrupts                      */
#define IRQ_NO_SC                	8		/* Touch sensing interrupt                               */
#define IRQ_NO_DMA1_CH1           	9		/* DMA1 channel 1 interrupt                              */
#define IRQ_NO_DMA_CH2_3            10      /* DMA channel 2 and 3 interrupts                        */
#define IRQ_NO_DMA2_CH1_2           10   	/* DMA2 channel 1 and 2 interrupts                       */
#define IRQ_NO_DMA_CH4_5_6_7        11      /* DMA channel 4, 5, 6 and 7 interrupts 				 */
#define IRQ_NO_DMA_CH4_3_4_5        11      /* DMA2 channel 3, 4 and 5 interrupts                    */
#define IRQ_NO_ADC_COMP           	12		/* ADC and comparator interrupts                         */
#define IRQ_NO_TIM1_BRK_UP_TRG_COM	13		/* TIM1 break, update, trigger and commutation interrupt */
#define IRQ_NO_TIM1_CC            	14		/* TIM1 Capture Compare interrupt                        */
#define IRQ_NO_TIM2               	15		/* TIM2 global interrupt                                 */
#define IRQ_NO_TIM3              	16		/* TIM3 global interrupt                                 */
#define IRQ_NO_TIM6_DAC          	17		/* TIM6 global interrupt and DAC underrun interrupt      */
#define IRQ_NO_TIM7               	18		/* TIM7 global interrupt                                 */
#define IRQ_NO_TIM14             	19		/* TIM14 global interrupt                                */
#define IRQ_NO_TIM15             	20		/* TIM15 global interrupt                                */
#define IRQ_NO_TIM16              	21		/* TIM16 global interrupt                                */
#define IRQ_NO_TIM17             	22		/* TIM17 global interrupt                                */
#define IRQ_NO_I2C1               	23		/* I2C1 global interrupt                                 */
#define IRQ_NO_I2C2              	24		/* I2C2 global interrupt                                 */
#define IRQ_NO_SPI1             	25  	/* SPI1_global_interrupt                                 */
#define IRQ_NO_SPI2             	26  	/* SPI2 global interrupt                                 */
#define IRQ_NO_USART1           	27  	/* USART1 global interrupt                               */
#define IRQ_NO_USART2           	28  	/* USART2 global interrupt                               */
#define IRQ_NO_USART3_4         	29  	/* USART3 and USART4 global interrupt                    */
#define IRQ_NO_CEC_CAN          	30  	/* CEC and CAN global interrupt                          */
#define IRQ_NO_USB					31      /* USB global interrupt									 */

/* Macro returns a value (between 0 - 7) for given GPIO base address(x)*/
#define GPIO_BASEADDR_TO_VALUE(x)    ( (x == GPIOA)?0:\
                                      (x == GPIOB)?1:\
                                      (x == GPIOC)?2:\
                                      (x == GPIOD)?3:\
                                      (x == GPIOE)?4:\
                                      (x == GPIOF)?5:0 )

/*
 * Macros for all possible priority levels
 */
#define NVIC_IRQ_PRI0       0
#define NVIC_IRQ_PRI1       1
#define NVIC_IRQ_PRI2       2
#define NVIC_IRQ_PRI3       3

/* Macros for states*/

#define ENABLE 				1
#define DISABLE				0
#define	SET					ENABLE
#define RESET				DISABLE


#endif /* INC_STM32F072B_H_ */
