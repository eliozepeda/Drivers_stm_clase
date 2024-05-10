/*
 * stm32l476xx_spi_driver.c
 *
 *  Created on: Apr 9, 2024
 *      Author: Elio Zepeda
 */


#include "stm32l476xx_spi_driver.h"

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);
/*Peripheral clock setup*/

/***************************************************************************************
 * @fn						-SPI_PeriClockControl
 *
 * @brief					-This functions enables or disables peripheral clock for the giving SPI
 *
 *
 * @pSPIx 					-Base address of the SPI peripheral
 * @EnorDi 					-Enable or disable MACRO
 *
 * @return 					- none
 */
void SPI_PeriClockControl(SPI_Reg_Def_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
			{
				SPI1_PERI_CLOCK_ENABLE();
			}
			else if (pSPIx == SPI2)
			{
				SPI2_PERI_CLOCK_ENABLE();
			}
			else if (pSPIx == SPI3)
			{
				SPI3_PERI_CLOCK_ENABLE();
			}
		}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PERI_CLOCK_DISABLE();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PERI_CLOCK_DISABLE();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PERI_CLOCK_DISABLE();
		}
	}
}

/*Init and De-init*/
/***************************************************************************************
 * @fn						- SPI_Init
 *
 * @brief					- Initializes the SPI
 *
 * @param SPI_Handle_t 		- Pointer to the SPI handle structure
 *
 * @return 					- None
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//first configure CR1 register
	uint32_t tempreg = 0;

	//1. Configurar el modo del dispositivo
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << MSTR_BIT;

	//2. Configurar el bus
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//Bit BIDI MODE debe ser 0 (2-line unidirectional data mode selected)
		tempreg &= ~(1<< BIDIMODE_BIT);
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//Bit BIDI MODE debe ser 1 (1-line bidirectional data mode selected)
		tempreg |= (1 << BIDIMODE_BIT);
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_S_RXONLY)
	{
		//Bit BIDI MODE debe ser 0 (2-line unidirectional data mode selected)
		//BIT RXONLY debe ser 1 (enables simplex communication using a single unidirectional line to receive data exclusively.)
		tempreg &= ~(1 << BIDIMODE_BIT);
		tempreg |= (1 << RXONLY_BIT);
	}

	//3. Configurar velocidad
	tempreg &= ~(3 <<  BR_BIT);
	tempreg |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << BR_BIT);

	//4. Configurar CLOCK POLARITY CPOL (estado del reloj en idle)
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPOL << CPOL_BIT);

	//5. Configurar CLOCK PHASE CPHA (en que flanco va a ser muestreada la información)
	tempreg |= (pSPIHandle->SPI_Config.SPI_CPHA << CPHA_BIT);

	//6. Configurar Slave Select ++++SW or HW?
	tempreg |= (pSPIHandle->SPI_Config.SPI_SSM << SSM_BIT);

	pSPIHandle->pSPIx->CR1 |= tempreg;

	//7 configure data size in CR2 register
	pSPIHandle->pSPIx->CR2 |= (pSPIHandle->SPI_Config.SPI_DS << DS_BIT);
	if(pSPIHandle->SPI_Config.SPI_DS == SPI_DS_8_BITS)
	{
		pSPIHandle->pSPIx->CR2 |= (THRESHOLD_8_BIT << FRXTH_BIT);
	}
}

/***************************************************************************************
 * @fn						- SPI_DeInit
 *
 * @brief					- De-initializes the SPI
 *
 * @param SPI_Reg_Def_t 	- Base address of the SPI peripheral
 *
 * @return 					- None
 */
void SPI_DeInit(SPI_Reg_Def_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_RESET();
	}
}

/*Data send and receive*/
/***************************************************************************************
 * @fn						- SPI_SendData
 *
 * @brief					- Send data to the bus
 *
 * @param *SPI_Reg_Def_t 	- Base address of the SPI peripheral
 * @param *pTxBuffer 		- Pointer to the buffer
 * @param Len 				- Length of data
 *
 * @return 					- None
 *
 * @Note					-This is blocking call
 */
void SPI_SendData(SPI_Reg_Def_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//Esperar a que TX flag sea 1
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET)
		{

		}

		if((pSPIx->CR2>>8 & 0xF) ==  SPI_DS_8_BITS) /*Evaluar si el data size es 8 bits*/
		{
			//8 bits data size
			*((__vo uint8_t*)&pSPIx->DR) = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}else if((pSPIx->CR2>>8 & 0xF) ==  SPI_DS_16_BITS) /*Evaluar si el data size es 16 bits*/
		{
			//16 bits data size
			uint16_t Data_swaped;
			Data_swaped = (*pTxBuffer << 8)| (*(pTxBuffer + 1));

			*((__vo uint16_t*)&pSPIx->DR) = Data_swaped;
			Len--;
			Len--;
			pTxBuffer+=2; //se incrementa 2 el Tx buffer ya que estamos mandando 2 bytes
		}
	}


}

/***************************************************************************************
 * @fn						- SPI_ReceiveData
 *
 * @brief					- Receive data from the bus
 *
 * @param *SPI_Reg_Def_t 	- Base address of the SPI peripheral
 * @param *pTxBuffer 		- Pointer to the buffer
 * @param Len 				- Length of data
 *
 * @return 					- None
 */
void SPI_ReceiveData(SPI_Reg_Def_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while((int8_t)Len > 0)
	{
		//Esperar a que RXNE sea 1
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET)
		{

		}
			if((pSPIx->CR2>>8 & 0xF) ==  SPI_DS_8_BITS) /*Evaluar si el data size es 8 bits*/
			{
				//8 bits data size
				//GUARDAR DATOS EN EL BUFFER
				*pTxBuffer = *((__vo uint8_t*)&pSPIx->DR);
				Len--;
				pTxBuffer++;
			}else if((pSPIx->CR2>>8 & 0xF) ==  SPI_DS_16_BITS) /*Evaluar si el data size es 16 bits*/
			{
				//16 bits data size
				uint16_t Data_swaped;

				Data_swaped = *((__vo uint16_t*)&pSPIx->DR);
				*pTxBuffer = (Data_swaped >> 8);
				*(pTxBuffer + 1) = (Data_swaped & 0xFF);
				Len--;
				Len--;
				pTxBuffer+=2; //se incrementa 2 el Tx buffer ya que estamos mandando 2 bytes
			}

	}
}

uint8_t SPI_GetFlagStatus(SPI_Reg_Def_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

void SPI_SSIConfig(SPI_Reg_Def_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SSI_BIT);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SSI_BIT);
	}
}

void SPI_SSOEConfig(SPI_Reg_Def_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SSOE_BIT);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SSOE_BIT);
	}
}

void SPI_PeripheralClontrol(SPI_Reg_Def_t *pSPIx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPE_BIT);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPE_BIT);
	}
}

/*IRQ configuration and ISR handling*/
/***************************************************************************************
 * @fn						- SPI_IRQInterruptConfig
 *
 * @brief					- Configures the SPI interrupt
 *
 * @param IRQNumber 		- Interrupt number
 * @param EnorDi 			- Enable or disable
 *
 * @return 					- None
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber %32));
		}
		else if(IRQNumber > 64 && IRQNumber < 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber %64));
		}
	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1 first find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_value = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + (iprx)) |= (IRQPriority << shift_value);
}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1.-Guardar la dirección del TX buffer en una variable global
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;

		//2.- Marcar el estado de SPI como ocupado en transmision, para que ningúng otro código pueda usar el mismo SPI.
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3.- Habilitar la interrupción TXEIE (a nivel del microcontrolador)
		pSPIHandle->pSPIx->CR2 |= (ENABLE << TXEIE_BIT);
	}



//4.- Transmision de datos se va a llevar acabo por el Interrupt Service Routine.( se implementará después )
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1.- Save the Tx buffer address and len information in some global variables
			pSPIHandle->pRxBuffer = pTxBuffer;
			pSPIHandle->RxLen = Len;
		//2.- Mark the SPI state as busy in transmission so that no other code can take over same
		//    SPI peripheral until transmission is over
			pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3.- Enable the RXNEIE control bit to get interrupt whenever TXE flag is set in SR
			pSPIHandle->pSPIx->CR2 |= (ENABLE << RXNEIE_BIT);
	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp1, temp2;

	//First check for TXE
	temp1 = pSPIHandle->pSPIx->SR & (1 << TXE_BIT);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << TXEIE_BIT);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	//check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & (1 << RXNE_BIT);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << RXNEIE_BIT);

	if(temp1 && temp2)
	{
		//handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	//check for ovr flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << OVR_BIT);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << ERRIE_BIT);
	if(temp1 && temp2)
	{
		//handle RXNE
		spi_ovr_interrupt_handle(pSPIHandle);
	}
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

		if((pSPIHandle->pSPIx->CR2>>8 & 0xF) ==  SPI_DS_8_BITS) /*Evaluar si el data size es 8 bits*/
		{
			//8 bits data size
			*((__vo uint8_t*)&pSPIHandle->pSPIx->DR) = *((uint8_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++;
		}else if((pSPIHandle->pSPIx->CR2>>8 & 0xF) ==  SPI_DS_16_BITS) /*Evaluar si el data size es 16 bits*/
		{
			//16 bits data size
			uint16_t Data_swaped;
			Data_swaped = (*((__vo uint16_t*)pSPIHandle->pTxBuffer) << 8| (*((__vo uint16_t*)pSPIHandle->pTxBuffer + 1)));


			*((__vo uint16_t*)&pSPIHandle->pSPIx->DR) = Data_swaped;
			pSPIHandle->TxLen--;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer+=2; //se incrementa 2 el Tx buffer ya que estamos mandando 2 bytes
		}

		if(!pSPIHandle->TxLen)
		{
			//Cerrar la comunicación
			SPI_CloseTransmission(pSPIHandle);
			//llamar al callback
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
		}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if((pSPIHandle->pSPIx->CR2>>8 & 0xF) == SPI_DS_8_BITS)
	{
		//8bits Data size
		//1. Guardar datos en BUFFER
		*((uint8_t*)pSPIHandle->pRxBuffer) = *((__vo uint8_t*)&pSPIHandle->pSPIx->DR);
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}else if((pSPIHandle->pSPIx->CR2>>8 & 0xF) == SPI_DS_16_BITS){
		//16bits

		uint16_t Data_swaped;
		//1. Guardar datos en BUFFER
		Data_swaped = *((__vo uint16_t*)&pSPIHandle->pSPIx->DR);
		*((__vo uint16_t*)pSPIHandle->pTxBuffer) = (Data_swaped >> 8);
		*((__vo uint16_t*)pSPIHandle->pTxBuffer + 1) = (Data_swaped & 0xFF);
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer+=2; //se incrementa 2 el Tx buffer ya que estamos mandando 2 bytes
	}


	if(!pSPIHandle->RxLen)
	{
		//Cerrar la comunicación
		SPI_CloseReception(pSPIHandle);
		//llamar al callback
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);

	}
}

static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	//llamar al callback
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	//deshabilitar interrupcion txeie
	pSPIHandle->pSPIx->CR2 &= ~(1 << TXEIE_BIT);
	//LIMPIAR A DONDE APUNTA MI VARIABLE Tx Buffer
	pSPIHandle->pTxBuffer = NULL;
	//Limpiar la variable de len
	pSPIHandle->TxLen = 0;
	//cambiar el estado del bus a ready
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << RXNEIE_BIT);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t APP_EVENT)
{
	//esta funcion debe de ser implementada en la aplicación
}
