/*
 * stm32f411retx.h
 *
 *  Created on: 7 dic 2025
 *      Author: octav
 */

#include <stdint.h>
#include <string.h>
#include "stm32f411xx_gpio_driver.h"
#include "stm32f411xx_spi_driver.h"

#ifndef INC_STM32F411RETX_H_
#define INC_STM32F411RETX_H_

#define __vo volatile

//ARM CORTEX Mx Processor NVIC ISERx Register Addr
#define NVIC_ISER0		( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1		( (__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2		( (__vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3		( (__vo uint32_t*) 0xE000E10C )

//ARM CORTEX Mx Processor NVIC ICERx Register Addr
#define NVIC_ICER0		( (__vo uint32_t*) 0XE000E180 )
#define NVIC_ICER1		( (__vo uint32_t*) 0XE000E184 )
#define NVIC_ICER2		( (__vo uint32_t*) 0XE000E188 )
#define NVIC_ICER3		( (__vo uint32_t*) 0XE000E18C )

//ARM CORTEX Mx Processor NVIC Priority Register Addr
#define NVIC_IPR_ADDR		( (__vo uint32_t*) 0xE000E400 )

#define NO_PR_BITS_IMP	4

#define FLASH_BASEADDR	0x08000000U  	/* Base Addres of Flash Memory */
#define SRAM_BASEADDR		0x20000000U		/* Base Addres of SRAM Memory */
#define ROM_BASEADDR		0x1FFF0000U		/* Base Addres of System Memory */

// AHBx and APBx Bus Peripheral base Addrs

#define PERP_ADDR		0X40000000U
#define APB1_ADDR		PERP_ADDR
#define APB2_ADDR		0X40010000U
#define AHB1_ADDR		0X40020000U
#define AHB2_ADDR		0X50000000U

// Base Addr of peripherals which are hanging on AHB1 bus

#define GPIOA_ADDR		(AHB1_ADDR + 0x0000)
#define GPIOB_ADDR		(AHB1_ADDR + 0x0400)
#define GPIOC_ADDR		(AHB1_ADDR + 0x0800)
#define GPIOD_ADDR		(AHB1_ADDR + 0x0C00)
#define GPIOE_ADDR		(AHB1_ADDR + 0x1000)
#define GPIOH_ADDR		(AHB1_ADDR + 0x0C00)
#define RCC_ADDR		(AHB1_ADDR + 0x3800)


// Base Addr of peripherals which are hanging on APB1 bus

#define I2C1_ADDR		(APB1_ADDR + 0x5400)
#define I2C2_ADDR		(APB1_ADDR + 0x5800)
#define I2C3_ADDR		(APB1_ADDR + 0x5C00)
#define USART2_ADDR	(APB1_ADDR + 0x4400)
#define SPI3_ADDR		(APB1_ADDR + 0x3C00)
#define SPI2_ADDR		(APB1_ADDR + 0x3800)

// Base Addr of peripherals which are hanging on APB2 bus

#define SPI1_ADDR		(APB2_ADDR + 0x3000)
#define SPI4_ADDR		(APB2_ADDR + 0x3400)
#define SPI5_ADDR		(APB2_ADDR + 0x5000)
#define EXT1_ADDR		(APB2_ADDR + 0x3C00)
#define SYSCFG_ADDR	(APB2_ADDR + 0x3800)
#define USART6_ADDR	(APB2_ADDR + 0x1400)
#define USART1_ADDR	(APB2_ADDR + 0x1000)

//Peripheral Register Definition Structures
//GPIO
typedef struct
{
	uint32_t MODER;		//GPIO port mode register
	uint32_t OTYPER;	//GPIO port output type register
	uint32_t OSPEEDR;	//GPIO port output speed register
	uint32_t PUPDR;		//GPIO port pull-up/pull-down register
	uint32_t IDR;		//GPIO port input data register
	uint32_t ODR;		//GPIO port output data register
	uint32_t BSRR;		//GPIO port bit set/reset register
	uint32_t LCKR;		//GPIO port configuration lock register
	uint32_t AFR[2];	//GPIO alternate function low[0] & high[1] register
}GPIOx_Reg_t;
//SPI
typedef struct{
	uint32_t CR1;
	uint32_t CR2;
	uint32_t SR;
	uint32_t DR;
	uint32_t CRCPR;
	uint32_t RXCRCR;
	uint32_t TXCRCR;
	uint32_t I2SCFGR;
	uint32_t I2SPR;
}SPI_RegDef_t;
//RCC
typedef struct
{
	uint32_t CR;
	uint32_t PLLCFGR;
	uint32_t CFGR;
	uint32_t CIR;
	uint32_t AHB1RSTR;
	uint32_t AHB2RSTR;
	uint32_t RESERVED0[2];
	uint32_t APB1RSTR;
	uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	uint32_t AHB1ENR;
	uint32_t AHB2ENR;
	uint32_t RESERVED2[2];
	uint32_t APB1ENR;
	uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	uint32_t AHB1LPENR;
	uint32_t AHB2LPENR;
	uint32_t RESERVED4[2];
	uint32_t APB1LPENR;
	uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	uint32_t BDCR;
	uint32_t CSR;
	uint32_t RESERVED6[2];
	uint32_t SSCGR;
	uint32_t PLLI2SCFGR;
	uint32_t RESERVED7;
	uint32_t DCKCFGR;

}RCC_Reg_t;
//EXTI
typedef struct
{
	uint32_t IMR;
	uint32_t EMR;
	uint32_t RTSR;
	uint32_t FTSR;
	uint32_t SWIER;
	uint32_t PR;
}EXTI_Reg_t;
//SYSCFG
typedef struct
{
	uint32_t MEMRMP;
	uint32_t PMC;
	uint32_t EXTICR[4];
	uint32_t CMPCR;
}SYSCFG_Reg_t;
//Peripheral Def

#define GPIOA				((GPIOx_Reg_t*) GPIOA_ADDR)
#define GPIOB				((GPIOx_Reg_t*) GPIOB_ADDR)
#define GPIOC				((GPIOx_Reg_t*) GPIOC_ADDR)
#define GPIOD				((GPIOx_Reg_t*) GPIOD_ADDR)
#define GPIOE				((GPIOx_Reg_t*) GPIOE_ADDR)
#define GPIOH				((GPIOx_Reg_t*) GPIOH_ADDR)

#define SPI1				((SPI_RegDef_t*) SPI1_ADDR)
#define SPI2				((SPI_RegDef_t*) SPI2_ADDR)
#define SPI3				((SPI_RegDef_t*) SPI3_ADDR)
#define SPI4				((SPI_RegDef_t*) SPI4_ADDR)
#define SPI5				((SPI_RegDef_t*) SPI5_ADDR)

#define RCC					((RCC_Reg_t*) RCC_ADDR)

#define EXTI				((EXTI_Reg_t*) EXT1_ADDR)

#define SYSCFG				((SYSCFG_Reg_t*) SYSCFG_ADDR)

//CLK EN Macros for GPIOx Peripherals

#define PA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define PB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define PC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define PD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define PE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define PH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))

//CLK EN Macros for I2Cx Peripherals

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

//CLK EN Macros for SPIx Peripherals

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()		(RCC->APB2ENR |= (1 << 20))

//CLK EN Macros for USARTx Peripherals

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

//CLK EN Macros for SYSCFG Peripheral

#define SYSCFG_PCKL_EN()	(RCC->APB2ENR |= (1 << 14))

//CLK DI Macros for GPIOx Peripherals

#define PA_PCLK_DI()			(RCC->AHB1ENR &= (1 << 0))
#define PB_PCLK_DI()			(RCC->AHB1ENR &= (1 << 1))
#define PC_PCLK_DI()			(RCC->AHB1ENR &= (1 << 2))
#define PD_PCLK_DI()			(RCC->AHB1ENR &= (1 << 3))
#define PE_PCLK_DI()			(RCC->AHB1ENR &= (1 << 4))
#define PH_PCLK_DI()			(RCC->AHB1ENR &= (1 << 7))

//CLK DI Macros for I2Cx Peripherals

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= (1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= (1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= (1 << 23))

//CLK DI Macros for SPIx Peripherals

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= (1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= (1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= (1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= (1 << 13))
#define SPI5_PCLK_DI()		(RCC->APB2ENR &= (1 << 20))

//CLK DI Macros for USARTx Peripherals

#define USART1_PCLK_DI()	(RCC->APB2ENR &= (1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= (1 << 17))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= (1 << 5))

//CLK DI Macros for SYSCFG Peripheral

#define SYSCFG_PCKL_DI()	(RCC->APB2ENR &= (1 << 14))

// Macros to reset the GPIOx Perihperals

#define PA_REG_RST()		do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0);
#define PB_REG_RST()		do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0);
#define PC_REG_RST()		do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0);
#define PD_REG_RST()		do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0);
#define PE_REG_RST()		do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0);
#define PH_REG_RST()		do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0);

#define GPIO_BASEADDR_2CODE(x)  ((x == GPIOA) ? 0 : \
									(x == GPIOB) ? 1 : \
									(x == GPIOC) ? 2 : \
									(x == GPIOD) ? 3 : \
									(x == GPIOE) ? 4 : \
									(x == GPIOH) ? 7 : 0 )

//IRQ (Interrupt Request) Numbers of STM32F411RETx MCU

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10		40

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    		0
#define NVIC_IRQ_PRI15    	15

//Generics

#define ENABLE				1
#define DISABLE			0
#define HIGH				1
#define LOW				0
#define SET				ENABLE
#define RESET				DISABLE
#define FLAG_RESET			RESET
#define FLAG_SET			SET

//Bit position definition of SPI Peripheral
//SPI_CR1
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE		6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI		8
#define SPI_CR1_SSM		9
#define SPI_CR1_RXONLY	10
#define SPI_CR1_DFF		11
#define SPI_CR1_CRCNEXT	12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE	14
#define SPI_CR1_BIDIMODE	15

//SPI_CR2
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE			2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE			7

//SPI_SR
#define SPI_SR_RXNE		0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF		5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

#endif /* INC_STM32F411RETX_H_ */
