/*
 * stm32l476xx.h
 *
 *  Created on: Mar 7, 2024
 *      Author: Elio Zepeda
 */

#ifndef INC_STM32L476XX_H_
#define INC_STM32L476XX_H_
#include <stdint.h>
#include <stddef.h>
#define __vo volatile
#define __weak __attribute__((weak))
/*CORTEX M4 Processor specific*/
/*Interrupt Set-enable Registers*/
#define NVIC_ISER0  		((__vo uint32_t*)0xE000E100U)
#define NVIC_ISER1  		((__vo uint32_t*)0xE000E104U)
#define NVIC_ISER2  		((__vo uint32_t*)0xE000E108U)
#define NVIC_ISER3  		((__vo uint32_t*)0xE000E10CU)

/*Interrupt Clear-enable Registers*/
#define NVIC_ICER0  		((__vo uint32_t*)0XE000E180U)
#define NVIC_ICER1  		((__vo uint32_t*)0XE000E184U)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188U)
#define NVIC_ICER3  		((__vo uint32_t*)0XE000E18CU)

/*Interrupt Priority Registers*/
#define NVIC_IPR_BASE_ADDR  ((__vo uint32_t*)0xE000E400U)

#define NO_PR_BITS_IMPLEMENTED 	4

/*---------------------BASE ADDRESSES START---------------------*/
/*Memories*/
/*All this definitions can be find in the page 73 of the reference manual
 * Figure 3. Memory map for STM32L476xx device*/
#define EZ_FLASH_BASEADDR 0x08000000U
#define EZ_SRAM1_BASEADDR 0x20000000U
#define EZ_SRAM2_BASEADDR 0x10000000U 	/*Is not contiguous to SRAM1 in this board*/
#define EZ_ROM_BASEADDR   0x1FFF0000U 	/*System memory*/

/*Peripheral Buses*/
#define EZ_APB1_BASEADDR 0x40000000U
#define EZ_APB2_BASEADDR 0x40010000U
#define EZ_AHB1_BASEADDR 0x40020000U
#define EZ_AHB2_BASEADDR 0x48000000U
#define EZ_AHB3_BASEADDR 0xA0000000U /*This is the base addr according to the manual reference 2.2.2*/

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define EZ_SPI2_BASEADDR 0x40003800U
#define EZ_SPI3_BASEADDR 0x40003C00U
#define EZ_I2C1_BASEADDR 0x40005400U
#define EZ_I2C2_BASEADDR 0x40005800U
#define EZ_I2C3_BASEADDR 0x40005C00U


/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */

#define EZ_SYSCFG_BASEADDR 0x40010000U
#define EZ_EXTI_BASEADDR 0x40010400U
#define EZ_SPI1_BASEADDR 0x40013000U

/*
 * Base addresses of peripherals which are hanging on AHB2 bus
 */

#define EZ_GPIOA_BASEADDR 0x48000000U
#define EZ_GPIOB_BASEADDR 0x48000400U
#define EZ_GPIOC_BASEADDR 0x48000800U
#define EZ_GPIOD_BASEADDR 0x48000C00U
#define EZ_GPIOE_BASEADDR 0x48001000U
#define EZ_GPIOF_BASEADDR 0x48001400U
#define EZ_GPIOG_BASEADDR 0x48001800U
#define EZ_GPIOH_BASEADDR 0x48001C00U

#define EZ_RCC_BASEADDR   0x40021000U

/*---------------------BASE ADDRESSES END---------------------*/


/************************Peripheral Register Definitions Structures****************************/
/*Note: Registers of a peripheral are specific to STM32L476xx MCU*/

/*Peripheral register definition structure for GPIO*/
typedef struct
{
	__vo uint32_t MODER;	/*Mode register. These bits are written by software to configure the I/O mode. 																				Address offset:0x00*/
	__vo uint32_t OTYPER;	/*Output type register. These bits are written by software to configure the I/O output type. 																Address offset:0x04*/
	__vo uint32_t OSPEEDR;	/*Output speed register. These bits are written by software to configure the I/O output speed. 																Address offset:0x08*/
	__vo uint32_t PUPDR;	/*Pull-up/pull-down register. These bits are written by software to configure the I/O pull-up or pull-down. 												Address offset:0x0C*/
	__vo uint32_t IDR;		/*Input data register. These bits are read-only. They contain the input value of the corresponding I/O port. 												Address offset:0x10*/
	__vo uint32_t ODR;		/*Output data register. These bits are used to control the I/O state of the I/O pins. 																		Address offset:0x14*/
	__vo uint32_t BSRR;		/*Bit set/reset register. These bits are used to set, reset the pins of the GPIO, write only. 																Address offset:0x18*/
	__vo uint32_t LCKR;		/*Configuration lock register. These bits are used to lock the configuration of the port bits when a correct write sequence is applied to bit 16 (LCKK). 	Address offset:0x1C*/
	__vo uint32_t AFR[2];	/*Alternate function register AFR[0] LOW REG AFR[1] HIGH REG. These bits are written by software to configure alternate function I/O. 						Address offset:0x20*/
	__vo uint32_t BRR;		/*Bit reset register. These bits are used to reset the corresponding ODx bit. 																				Address offset:0x28*/
	__vo uint32_t ASCR;		/*Analog switch control register. These bits are written by software to configure the analog connection of the IO											Address offset:0x2C*/
}GPIO_Reg_Def_t;

/*Peripheral register definition structure for RCC*/
typedef struct
{
	__vo uint32_t CR;	    		/*Clock control register. These bits are written by software to TURN ON the clocks. 																			Address offset:0x00*/
	__vo uint32_t ICSCR;			/*Internal clock sources calibration register. These bits are written by software to configure the clock calibration. 											Address offset:0x04*/
	__vo uint32_t CFGR;	    		/*Clock configuration register. These bits are written by software to select the clock source. 																	Address offset:0x08*/
	__vo uint32_t PLLCFGR;			/*PLL configuration register. These bits are written by software to configure the PLL clock outputs according to the formulas in RM. 							Address offset:0x0C*/
	__vo uint32_t PLLSAI1CFGR;		/*PLLSAI1 configuration register. These bits are written by software to configure the PLLSAI1 clock outputs according to the formulas in RM. 					Address offset:0x10*/
	__vo uint32_t PLLSAI2CFGR;		/*PLLSAI2 configuration register. These bits are written by software to configure the PLLSAI2 clock outputs according to the formulas in RM. 					Address offset:0x14*/
	__vo uint32_t CIER;				/*Clock interrupt enable register. These bits are written by software to configure the clock interrupts. 														Address offset:0x18*/
	__vo uint32_t CIFR;				/*Clock interrupt flag register. These bits are written by hardware to know the state of a clock interrupt. 													Address offset:0x1C*/
	__vo uint32_t CICR;				/*Clock interrupt clear register. These bits are written by software to clear clock interrupts. 																Address offset:0x20*/
	__vo uint32_t RCC_RESERVED_1;	/*This 4 bytes are reserved. 																																	Address offset:0x24*/
	__vo uint32_t AHB1RSTR;			/*AHB1 peripheral reset register. These bits are written by software to reset a peripheral connected to AHB1. 													Address offset:0x28*/
	__vo uint32_t AHB2RSTR;			/*AHB2 peripheral reset register. These bits are written by software to reset a peripheral connected to AHB2. 													Address offset:0x2C*/
	__vo uint32_t AHB3RSTR;			/*AHB3 peripheral reset register. These bits are written by software to reset a peripheral connected to AHB3. 													Address offset:0x30*/
	__vo uint32_t RCC_RESERVED_2;	/*This 4 bytes are reserved. 																																	Address offset:0x34*/
	__vo uint32_t APB1RSTR1;		/*APB1 peripheral reset register 1. These bits are written by software to reset a peripheral connected to APB1. 												Address offset:0x38*/
	__vo uint32_t APB1RSTR2;		/*APB1 peripheral reset register 2. These bits are written by software to reset a peripheral connected to APB1. 												Address offset:0x3C*/
	__vo uint32_t APB2RSTR;			/*APB2 peripheral reset register. These bits are written by software to reset a peripheral connected to APB2. 													Address offset:0x40*/
	__vo uint32_t RCC_RESERVED_3;	/*This 4 bytes are reserved. 																																	Address offset:0x44*/
	__vo uint32_t AHB1ENR;			/*AHB1 peripheral clock enable register. These bits are written by software to enable a peripheral connected to AHB1. 											Address offset:0x48*/
	__vo uint32_t AHB2ENR;			/*AHB2 peripheral clock enable register. These bits are written by software to enable a peripheral connected to AHB2. 											Address offset:0x4C*/
	__vo uint32_t AHB3ENR;			/*AHB3 peripheral clock enable register. These bits are written by software to enable a peripheral connected to AHB3. 											Address offset:0x50*/
	__vo uint32_t RCC_RESERVED_4;	/*This 4 bytes are reserved. 																																	Address offset:0x54*/
	__vo uint32_t APB1ENR1;			/*APB1 peripheral clock enable register 1. These bits are written by software to enable a peripheral connected to APB1. 										Address offset:0x58*/
	__vo uint32_t APB1ENR2;			/*APB1 peripheral clock enable register 2. These bits are written by software to enable a peripheral connected to APB1. 										Address offset:0x5C*/
	__vo uint32_t APB2ENR;			/*APB2 peripheral clock enable register. These bits are written by software to enable a peripheral connected to APB2. 											Address offset:0x60*/
	__vo uint32_t RCC_RESERVED_5;	/*This 4 bytes are reserved. 																																	Address offset:0x64*/
	__vo uint32_t AHB1SMENR;		/*AHB1 peripheral clocks enable in Sleep and Stop modes register. These bits are written by software to enable a peripheral connected to AHB1 in sleep mode. 	Address offset:0x68*/
	__vo uint32_t AHB2SMENR;		/*AHB2 peripheral clocks enable in Sleep and Stop modes register. These bits are written by software to enable a peripheral connected to AHB2 in sleep mode. 	Address offset:0x6C*/
	__vo uint32_t AHB3SMENR;		/*AHB3 peripheral clocks enable in Sleep and Stop modes register. These bits are written by software to enable a peripheral connected to AHB3 in sleep mode. 	Address offset:0x70*/
	__vo uint32_t RCC_RESERVED_6;	/*This 4 bytes are reserved. 																																	Address offset:0x74*/
	__vo uint32_t APB1SMENR1;		/*APB1 peripheral clocks enable in Sleep and Stop modes register 1. These bits are written by software to enable a peripheral connected to APB1 in sleep mode. 	Address offset:0x78*/
	__vo uint32_t APB1SMENR2;		/*APB1 peripheral clocks enable in Sleep and Stop modes register 2. These bits are written by software to enable a peripheral connected to APB1 in sleep mode. 	Address offset:0x7C*/
	__vo uint32_t APB2SMENR;		/*APB2 peripheral clocks enable in Sleep and Stop modes register. These bits are written by software to enable a peripheral connected to APB2 in sleep mode.	Address offset:0x80*/
	__vo uint32_t RCC_RESERVED_7;	/*This 4 bytes are reserved. 																																	Address offset:0x84*/
	__vo uint32_t CCIPR;			/*Peripherals independent clock configuration register. These bits are written by software to configure independent clocks. 									Address offset:0x88*/
	__vo uint32_t RCC_RESERVED_8;	/*This 4 bytes are reserved. 																																	Address offset:0x8C*/
	__vo uint32_t BDCR;				/*Backup domain control register. These bits are written by software to configure backup. 																		Address offset:0x90*/
	__vo uint32_t CSR;				/*Control/status register. These bits are written by software and hardware to read and remove reset flags. 														Address offset:0x94*/
	__vo uint32_t CRRCR;			/*Clock recovery RC register. These bits are written by software and hardware to manage clock recovery. 														Address offset:0x98*/
	__vo uint32_t CCIPR2;			/*Peripherals independent clock configuration register2. These bits are written by software to configure independent clocks. 									Address offset:0x9C*/

}RCC_Reg_Def_t;



/*Peripheral register definition structure for EXTI*/
typedef struct
{
	__vo uint32_t IMR1;        /*!< EXTI Interrupt mask register 1,             Address offset: 0x00 */
	__vo uint32_t EMR1;        /*!< EXTI Event mask register 1,                 Address offset: 0x04 */
	__vo uint32_t RTSR1;       /*!< EXTI Rising trigger selection register 1,   Address offset: 0x08 */
	__vo uint32_t FTSR1;       /*!< EXTI Falling trigger selection register 1,  Address offset: 0x0C */
	__vo uint32_t SWIER1;      /*!< EXTI Software interrupt event register 1,   Address offset: 0x10 */
	__vo uint32_t PR1;         /*!< EXTI Pending register 1,                    Address offset: 0x14 */
	__vo uint32_t RESERVED1;   /*!< Reserved, 0x18                                                   */
	__vo uint32_t RESERVED2;   /*!< Reserved, 0x1C                                                   */
	__vo uint32_t IMR2;        /*!< EXTI Interrupt mask register 2,             Address offset: 0x20 */
	__vo uint32_t EMR2;        /*!< EXTI Event mask register 2,                 Address offset: 0x24 */
	__vo uint32_t RTSR2;       /*!< EXTI Rising trigger selection register 2,   Address offset: 0x28 */
	__vo uint32_t FTSR2;       /*!< EXTI Falling trigger selection register 2,  Address offset: 0x2C */
	__vo uint32_t SWIER2;      /*!< EXTI Software interrupt event register 2,   Address offset: 0x30 */
	__vo uint32_t PR2;         /*!< EXTI Pending register 2,                    Address offset: 0x34 */
}EXTI_Reg_Def_t;

typedef struct
{
  __vo uint32_t MEMRMP;      /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  __vo uint32_t CFGR1;       /*!< SYSCFG configuration register 1,                   Address offset: 0x04      */
  __vo uint32_t EXTICR[4];   /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  __vo uint32_t SCSR;        /*!< SYSCFG SRAM2 control and status register,          Address offset: 0x18      */
  __vo uint32_t CFGR2;       /*!< SYSCFG configuration register 2,                   Address offset: 0x1C      */
  __vo uint32_t SWPR;        /*!< SYSCFG SRAM2 write protection register,            Address offset: 0x20      */
  __vo uint32_t SKR;         /*!< SYSCFG SRAM2 key register,                         Address offset: 0x24      */
} SYSCFG_Reg_Def_t;

typedef struct
{
	__vo uint32_t CR1;         /*!< SPI Control register 1,                              Address offset: 0x00 */
	__vo uint32_t CR2;         /*!< SPI Control register 2,                              Address offset: 0x04 */
	__vo uint32_t SR;          /*!< SPI Status register,                                 Address offset: 0x08 */
	__vo uint32_t DR;          /*!< SPI data register,                                   Address offset: 0x0C */
	__vo uint32_t CRCPR;       /*!< SPI CRC polynomial register,                         Address offset: 0x10 */
	__vo uint32_t RXCRCR;      /*!< SPI Rx CRC register,                                 Address offset: 0x14 */
	__vo uint32_t TXCRCR;      /*!< SPI Tx CRC register,                                 Address offset: 0x18 */
}SPI_Reg_Def_t;

/*
 * Peripheral definitions ( Peripheral base addresses typecasted to xxx_RegDef_t )
 * */
#define GPIOA ((GPIO_Reg_Def_t*)EZ_GPIOA_BASEADDR)
#define GPIOB ((GPIO_Reg_Def_t*)EZ_GPIOB_BASEADDR)
#define GPIOC ((GPIO_Reg_Def_t*)EZ_GPIOC_BASEADDR)
#define GPIOD ((GPIO_Reg_Def_t*)EZ_GPIOD_BASEADDR)
#define GPIOE ((GPIO_Reg_Def_t*)EZ_GPIOE_BASEADDR)
#define GPIOF ((GPIO_Reg_Def_t*)EZ_GPIOF_BASEADDR)
#define GPIOG ((GPIO_Reg_Def_t*)EZ_GPIOG_BASEADDR)
#define GPIOH ((GPIO_Reg_Def_t*)EZ_GPIOH_BASEADDR)

/*RCC definition*/
#define RCC   ((RCC_Reg_Def_t*)EZ_RCC_BASEADDR)


/*EXTI definition*/
#define EXTI   ((EXTI_Reg_Def_t*)EZ_EXTI_BASEADDR)

/*SYSCFG definition*/
#define SYSCFG   ((SYSCFG_Reg_Def_t*)EZ_SYSCFG_BASEADDR)

/*SPI definition*/
#define SPI1   ((SPI_Reg_Def_t*)EZ_SPI1_BASEADDR)
#define SPI2   ((SPI_Reg_Def_t*)EZ_SPI2_BASEADDR)
#define SPI3   ((SPI_Reg_Def_t*)EZ_SPI3_BASEADDR)

/*
 * Peripheral Clock Enable Macros
 * */

/*Clock Enable Macros for GPIOx peripherals*/
#define GPIOA_PERI_CLOCK_ENABLE()	(RCC->AHB2ENR |= (1 << 0))
#define GPIOB_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 1))
#define GPIOC_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 2))
#define GPIOD_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 3))
#define GPIOE_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 4))
#define GPIOF_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 5))
#define GPIOG_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 6))
#define GPIOH_PERI_CLOCK_ENABLE() 	(RCC->AHB2ENR |= (1 << 7))

/*Clock Enable Macros for I2Cx peripherals*/
#define I2C1_PERI_CLOCK_ENABLE() 	(RCC->APB1ENR1 |= (1 << 21))
#define I2C2_PERI_CLOCK_ENABLE() 	(RCC->APB1ENR1 |= (1 << 22))
#define I2C3_PERI_CLOCK_ENABLE() 	(RCC->APB1ENR1 |= (1 << 23))

/*Clock Enable Macros for SPIx peripherals*/
#define SPI1_PERI_CLOCK_ENABLE() 	(RCC->APB2ENR  |= (1 << 12))
#define SPI2_PERI_CLOCK_ENABLE() 	(RCC->APB1ENR1 |= (1 << 14))
#define SPI3_PERI_CLOCK_ENABLE() 	(RCC->APB1ENR1 |= (1 << 15))

/*Clock Enable Macros for SYSCFG peripheral*/
#define SYSCFG_PERI_CLOCK_ENABLE() 	(RCC->APB2ENR  |= (1 << 0))

#define GPIOA_RESET()	do{(RCC->AHB2RSTR |= (1 << 0)); (RCC->AHB2RSTR &= ~(1 << 0));} while(0)
#define GPIOB_RESET() 	do{(RCC->AHB2RSTR |= (1 << 1)); (RCC->AHB2RSTR &= ~(1 << 1));} while(0)
#define GPIOC_RESET() 	do{(RCC->AHB2RSTR |= (1 << 2)); (RCC->AHB2RSTR &= ~(1 << 2));} while(0)
#define GPIOD_RESET() 	do{(RCC->AHB2RSTR |= (1 << 3)); (RCC->AHB2RSTR &= ~(1 << 3));} while(0)
#define GPIOE_RESET() 	do{(RCC->AHB2RSTR |= (1 << 4)); (RCC->AHB2RSTR &= ~(1 << 4));} while(0)
#define GPIOF_RESET() 	do{(RCC->AHB2RSTR |= (1 << 5)); (RCC->AHB2RSTR &= ~(1 << 5));} while(0)
#define GPIOG_RESET() 	do{(RCC->AHB2RSTR |= (1 << 6)); (RCC->AHB2RSTR &= ~(1 << 6));} while(0)
#define GPIOH_RESET() 	do{(RCC->AHB2RSTR |= (1 << 7)); (RCC->AHB2RSTR &= ~(1 << 7));} while(0)

/*Reset Macros for SPIx peripherals*/
#define SPI1_RESET() 	do{(RCC->APB2RSTR |= (1 << 12));   (RCC->APB2RSTR &= ~(1 << 12));} while(0)
#define SPI2_RESET() 	do{(RCC->APB1RSTR1 |= (1 << 14)); (RCC->APB1RSTR1 &= ~(1 << 14));} while(0)
#define SPI3_RESET() 	do{(RCC->APB1RSTR1 |= (1 << 15)); (RCC->APB1RSTR1 &= ~(1 << 15));} while(0)


/*
 * Peripheral Clock Disable Macros
 * */
/*Clock Disable Macros for GPIOx peripherals*/
#define GPIOA_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 0))
#define GPIOB_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 1))
#define GPIOC_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 2))
#define GPIOD_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 3))
#define GPIOE_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 4))
#define GPIOF_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 5))
#define GPIOG_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 6))
#define GPIOH_PERI_CLOCK_DISABLE() 	(RCC->AHB2ENR &= ~(1 << 7))

/*Clock Disable Macros for I2Cx peripherals*/
#define I2C1_PERI_CLOCK_DISABLE() 	(RCC->APB1ENR1 &= ~(1 << 21))
#define I2C2_PERI_CLOCK_DISABLE() 	(RCC->APB1ENR1 &= ~(1 << 22))
#define I2C3_PERI_CLOCK_DISABLE() 	(RCC->APB1ENR1 &= ~(1 << 23))

/*Clock Disable Macros for SPIx peripherals*/
#define SPI1_PERI_CLOCK_DISABLE() 	(RCC->APB2ENR  &= ~(1 << 12))
#define SPI2_PERI_CLOCK_DISABLE() 	(RCC->APB1ENR1 &= ~(1 << 14))
#define SPI3_PERI_CLOCK_DISABLE() 	(RCC->APB1ENR1 &= ~(1 << 15))

/*General Macros Definitions*/
#define ENABLE 1
#define DISABLE 0
#define SET ENABLE
#define RESET DISABLE
#define FLAG_SET SET
#define FLAG_RESET RESET

#define HSI_ON() 			(RCC->CR |= (1 << 8))
#define SELECT_HSI_4MHZ() 	do {(RCC->CFGR &= ~(127 << 24)); (RCC->CFGR |= (35 << 24));} while(0)

#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)? 0 :\
									 (x == GPIOB)? 1 :\
									 (x == GPIOC)? 2 :\
									 (x == GPIOD)? 3 :\
									 (x == GPIOE)? 4 :\
									 (x == GPIOF)? 5 :\
									 (x == GPIOG)? 6 :\
									 (x == GPIOH)? 7 : 0 )

/******  STM32 specific Interrupt Numbers **********************************************************************/
#define  EXTI1_IRQn       7      /*!< EXTI Line1 Interrupt*/
#define  EXTI15_10_IRQn	  40
#define  EXTI0_IRQn		  6

//SPI
#define SPI1_GLOBAL_INTERRUPT	35
#define SPI2_GLOBAL_INTERRUPT	36
#define SPI3_GLOBAL_INTERRUPT	51

/******  STM32 specific Interrupt Priorities **********************************************************************/
#define  NVIC_IRQ_PRI0    0
#define  NVIC_IRQ_PRI1    1
#define  NVIC_IRQ_PRI2    2
#define  NVIC_IRQ_PRI3    3
#define  NVIC_IRQ_PRI4    4
#define  NVIC_IRQ_PRI5    5
#define  NVIC_IRQ_PRI6    6
#define  NVIC_IRQ_PRI7    7
#define  NVIC_IRQ_PRI8    8
#define  NVIC_IRQ_PRI9    9
#define  NVIC_IRQ_PRI10   10
#define  NVIC_IRQ_PRI11   11
#define  NVIC_IRQ_PRI12   12
#define  NVIC_IRQ_PRI13   13
#define  NVIC_IRQ_PRI14   14
#define  NVIC_IRQ_PRI15   15

#endif /* INC_STM32L476XX_H_ */
