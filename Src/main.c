/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <string.h>
#include "stm32l476xx.h"
#include "stm32l476xx_gpio_driver.h"
#include "stm32l476xx_spi_driver.h"

void Clock_Init(void);
void GPIOx_Init(void);
void Delay(void);
void SPI2_Inits(void);
#define MAX_LEN 500
//SPI2 MOSI --> PB15
//SPI2 MISO --> PB14
//SPI2 SCK -->  PB13
//SPI2 NSS -->  PB12
//ALTERNATE FUNCTION 5

int Data_Available = 0; //bandera que indica que hay un dato para leer
int Receive_Stop = 0; //bandera que indica que hay un dato para leer
SPI_Handle_t SPI2Handle;
char read_byte;
char receive_data[MAX_LEN];
int main(void)
{
	uint8_t dummy_write = 0XFF;
	Clock_Init();
	GPIOx_Init();

	GPIO_IRQInterruptConfig(EXTI0_IRQn,ENABLE); //señal de interrupcion
	GPIO_IRQPriorityConfig(EXTI0_IRQn, NVIC_IRQ_PRI15); //prioridad de la interrupción

	SPI_IRQInterruptConfig(SPI2_GLOBAL_INTERRUPT,ENABLE);
	SPI_IRQPriorityConfig(SPI2_GLOBAL_INTERRUPT, NVIC_IRQ_PRI15);

	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE); /*The NSS pin is managed by the hardware. The NSS signal
										is driven low as soon as the SPI is enabled in master mode (SPE=1), and is kept
										low until the SPI is disabled (SPE =0).*/

	char expected_data[] = "ELIO"; // Datos esperados

	while(1)
	{
		Receive_Stop = 0;

		int result = strcmp(receive_data,expected_data);
		if(result == 0)
		{
			//GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
			strcpy(receive_data," ");
		}

		while(!Data_Available);

		//GPIO_IRQInterruptConfig(EXTI0_IRQn,DISABLE); //Deshabilitamos interrupcion externa mientras recivimos el dato

		//SPI_PeripheralClontrol(SPI2,ENABLE); /*Habilitando SPI*/

		while(!Receive_Stop)
		{
			while( SPI_SendDataIT(&SPI2Handle, &dummy_write, 1) == SPI_BUSY_IN_TX);
			while( SPI_ReceiveDataIT(&SPI2Handle, (uint8_t *)&read_byte, 1) == SPI_BUSY_IN_RX);

		}


		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		//SPI_PeripheralClontrol(SPI2,DISABLE); /*Deshabilitando SPI*/

		Data_Available = 0;

		GPIO_IRQInterruptConfig(EXTI0_IRQn,ENABLE);
	}
}

void Clock_Init(void)
{
	HSI_ON();
	SELECT_HSI_4MHZ();
}

void GPIOx_Init(void)
{

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Handle_t GPIO_LED_struct;

	memset(&GPIO_LED_struct,0,sizeof(GPIO_LED_struct));
	GPIO_LED_struct.pGPIOx = GPIOA;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_LED_struct.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

	GPIO_Init(&GPIO_LED_struct); /*Initializing LED*/


	GPIO_Handle_t Interrupt_Signal_struct;

	memset(&Interrupt_Signal_struct,0,sizeof(Interrupt_Signal_struct));
	Interrupt_Signal_struct.pGPIOx = GPIOA;
	Interrupt_Signal_struct.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
	Interrupt_Signal_struct.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	Interrupt_Signal_struct.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	Interrupt_Signal_struct.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	Interrupt_Signal_struct.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

	GPIO_Init(&Interrupt_Signal_struct); /*Initializing LEDnterrupt_Signal*/


	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Handle_t SPI_PIN_CONFIG;
	memset(&SPI_PIN_CONFIG, 0, sizeof(SPI_PIN_CONFIG));
	SPI_PIN_CONFIG.pGPIOx = GPIOB;
	SPI_PIN_CONFIG.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_PIN_CONFIG.GPIO_PinConfig.GPIO_PinAltFunMode = AFN_5;
	SPI_PIN_CONFIG.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_PIN_CONFIG.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPI_PIN_CONFIG.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//MOSI
	SPI_PIN_CONFIG.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPI_PIN_CONFIG); //CONFIGURANDO MOSI

	//MISO
	SPI_PIN_CONFIG.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPI_PIN_CONFIG); /*Initializing MISO*/

	//SCK
	SPI_PIN_CONFIG.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPI_PIN_CONFIG); //CONFIGURANDO SCK

	//NSS
	SPI_PIN_CONFIG.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPI_PIN_CONFIG); //CONFIGURANDO NSS
}

void EXTI0_IRQHandler(void)
{
	Delay(); //200 ms
	GPIO_IRQHandling(GPIO_PIN_0);
	Data_Available = 1;
}

void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2Handle);
}

void Delay(void)
{
	for(uint32_t i = 0 ; i < 50000 ; i++);
}

void SPI2_Inits(void)
{


	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPI_Config.SPI_DS = SPI_DS_8_BITS;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_HW;
	//CONFIGURACION DE SPI2
	SPI_Init(&SPI2Handle);
}

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0;

	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
		receive_data[i++] = read_byte;

		if(read_byte == '$' || (i == MAX_LEN))
		{
			Receive_Stop = 1;
			receive_data[i-1] = '\0';
			i=0;
		}
	}

}
