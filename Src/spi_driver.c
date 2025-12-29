/*
 * spi_driver.c
 *
 *  Created on: Dec 29, 2025
 *      Author: wiki
 */

#include "spi_driver.h"

// Private helper functions
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPI_Handle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPI_Handle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPI_Handle);

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 * @brief             - Enables or Disables Peripheral Clock
 */
void SPI_PeriClockControl(SPI_TypeDef *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
	}
	else
	{
		if (pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 * @brief             - Initializes SPI Parameters
 */
void SPI_Init(SPI_Handle_t *pSPI_Handle)
{
	// Enable the Clock
	SPI_PeriClockControl(pSPI_Handle->pSPIx, ENABLE);

	// TEMPREG for CR1
	uint32_t tempreg_cr1 = 0;

	// 1. Configure the device mode
	// CORRECTION: Must use _Pos macros, not the Mask directly
	tempreg_cr1 |= (pSPI_Handle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_Pos);

	// 2. Configure the bus config
	if(pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// Bidi mode should be cleared
		tempreg_cr1 &= ~(1 << SPI_CR1_BIDIMODE_Pos);
	}
	else if (pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// Bidi mode should be set
		tempreg_cr1 |= (1 << SPI_CR1_BIDIMODE_Pos);
	}
	else if (pSPI_Handle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode should be cleared
		tempreg_cr1 &= ~(1 << SPI_CR1_BIDIMODE_Pos);
		// RXONLY bit must be set
		tempreg_cr1 |= (1 << SPI_CR1_RXONLY_Pos);
	}

	// 3. Configure the spi serial clock speed (baud rate)
	tempreg_cr1 |= (pSPI_Handle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR_Pos);

	// 4. Configure CPOL
	tempreg_cr1 |= (pSPI_Handle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_Pos);

	// 5. Configure CPHA
	tempreg_cr1 |= (pSPI_Handle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_Pos);

	// 6. Configure SSM
	tempreg_cr1 |= (pSPI_Handle->SPIConfig.SPI_SSM << SPI_CR1_SSM_Pos);

	// Apply configuration to CR1
	pSPI_Handle->pSPIx->CR1 = tempreg_cr1;

	// -----------------------------------------------------------
	// 7. Configure DFF (Data Size) in CR2
	// CORRECTION: STM32C0 uses CR2 DS[3:0] bits, not CR1 DFF
	// -----------------------------------------------------------
	uint32_t tempreg_cr2 = 0;

	if(pSPI_Handle->SPIConfig.SPI_DFF == SPI_DFF_16BITS)
	{
		// 16 Bit: DS = 1111 (0xF)
		tempreg_cr2 |= (0xF << SPI_CR2_DS_Pos);
	}
	else
	{
		// 8 Bit: DS = 0111 (0x7)
		tempreg_cr2 |= (0x7 << SPI_CR2_DS_Pos);
	}

	// We use Read-Modify-Write for CR2 to preserve other bits if needed,
	// but here we are initializing, so direct assignment is okay OR |= logic
	pSPI_Handle->pSPIx->CR2 &= ~(0xF << SPI_CR2_DS_Pos); // Clear old DS
	pSPI_Handle->pSPIx->CR2 |= tempreg_cr2;
}

void SPI_DeInit(SPI_TypeDef *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_TypeDef *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_SendData
 * @brief             - Blocking Call to send data
 */
void SPI_SendData(SPI_TypeDef *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. Wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check the Data Size (DS) in CR2
		// DS is bits [11:8]. If DS=0xF (15), it is 16-bit. If DS=0x7 (7), it is 8-bit.
		// We check if DS > 7 (meaning more than 8 bits)
		if ( (pSPIx->CR2 & SPI_CR2_DS_Msk) > (0x7 << SPI_CR2_DS_Pos) )
		{
			// 16 bit DFF
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			// CORRECTION: Increase pointer by 2 bytes
			pTxBuffer += 2;
		}
		else
		{
			// 8 bit DFF
			// Cast to uint8_t pointer to force 8-bit write
			*(__IO uint8_t *)&pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 * @brief             - Blocking Call to receive data
 */
void SPI_ReceiveData(SPI_TypeDef *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. Wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET);

		// 2. Check the Data Size (DS) in CR2
		if ( (pSPIx->CR2 & SPI_CR2_DS_Msk) > (0x7 << SPI_CR2_DS_Pos) )
		{
			// 16 bit DFF
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			// CORRECTION: Increase pointer by 2 bytes
			pRxBuffer += 2;
		}
		else
		{
			// 8 bit DFF
			*(pRxBuffer) = *(__IO uint8_t *)&pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}
}

void SPI_PeripheralControl(SPI_TypeDef *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE_Pos);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE_Pos);
	}
}

void SPI_SSIConfig(SPI_TypeDef *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI_Pos);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI_Pos);
	}
}

void SPI_SSOEConfig(SPI_TypeDef *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE_Pos);
	}
	else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE_Pos);
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*(volatile uint32_t*)NVIC_ISER0 |= (1 << IRQNumber);
		}
		// STM32C0 usually only has ISER0 (Start check for other series)
		#if defined(NVIC_ISER1)
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*(volatile uint32_t*)NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		#endif
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*(volatile uint32_t*)NVIC_ICER0 |= (1 << IRQNumber);
		}
		#if defined(NVIC_ICER1)
		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			*(volatile uint32_t*)NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		#endif
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	// In Cortex M0+, priority bits are usually top 2 bits (Check datasheet)
	// Assuming 2 bits implemented (Bits 7 and 6)
	uint8_t shift_amount = (8 * iprx_section) + (8 - __NVIC_PRIO_BITS);

	volatile uint32_t* pIPR = (volatile uint32_t*)NVIC_IPR; // Use header def or address

	pIPR[iprx] |= (IRQPriority << shift_amount);
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPI_Handle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		// 1. Save the Tx buffer address and Len information in handle
		pSPI_Handle->pTxBuffer = pTxBuffer;
		pSPI_Handle->TxLen = Len;

		// 2. Mark the SPI state as busy in transmission
		pSPI_Handle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit
		pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE_Pos);
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPI_Handle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPI_Handle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		// 1. Save the Rx buffer address and Len information
		pSPI_Handle->pRxBuffer = pRxBuffer;
		pSPI_Handle->RxLen = Len;

		// 2. Mark the SPI state as busy in reception
		pSPI_Handle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit
		pSPI_Handle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE_Pos);
	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1 , temp2;

	// Check for TXE
	temp1 = pHandle->pSPIx->SR & SPI_SR_TXE;
	temp2 = pHandle->pSPIx->CR2 & SPI_CR2_TXEIE;

	if(temp1 && temp2)
	{
		spi_txe_interrupt_handle(pHandle);
	}

	// Check for RXNE
	temp1 = pHandle->pSPIx->SR & SPI_SR_RXNE;
	temp2 = pHandle->pSPIx->CR2 & SPI_CR2_RXNEIE;

	if(temp1 && temp2)
	{
		spi_rxne_interrupt_handle(pHandle);
	}

	// Check for OVR flag
	temp1 = pHandle->pSPIx->SR & SPI_SR_OVR;
	temp2 = pHandle->pSPIx->CR2 & SPI_CR2_ERRIE;

	if(temp1 && temp2)
	{
		spi_ovr_err_interrupt_handle(pHandle);
	}
}

// ---------------- Helper Function Implementations ----------------

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPI_Handle)
{
	// Check Data Size (DS) in CR2
	if ( (pSPI_Handle->pSPIx->CR2 & SPI_CR2_DS_Msk) > (0x7 << SPI_CR2_DS_Pos) )
	{
		// 16 bit
		pSPI_Handle->pSPIx->DR = *((uint16_t*)pSPI_Handle->pTxBuffer);
		pSPI_Handle->TxLen--;
		pSPI_Handle->TxLen--;
		pSPI_Handle->pTxBuffer += 2;
	}
	else
	{
		// 8 bit
		*(__IO uint8_t *)&pSPI_Handle->pSPIx->DR = *pSPI_Handle->pTxBuffer;
		pSPI_Handle->TxLen--;
		pSPI_Handle->pTxBuffer++;
	}

	if(! pSPI_Handle->TxLen)
	{
		// TxLen is zero, close transmission
		SPI_CloseTransmisson(pSPI_Handle);
		SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPI_Handle)
{
	// Check Data Size (DS) in CR2
	if ( (pSPI_Handle->pSPIx->CR2 & SPI_CR2_DS_Msk) > (0x7 << SPI_CR2_DS_Pos) )
	{
		// 16 bit
		*((uint16_t*)pSPI_Handle->pRxBuffer) = (uint16_t)pSPI_Handle->pSPIx->DR;
		pSPI_Handle->RxLen -= 2;
		pSPI_Handle->pRxBuffer += 2;
	}
	else
	{
		// 8 bit
		*(pSPI_Handle->pRxBuffer) = *(__IO uint8_t *)&pSPI_Handle->pSPIx->DR;
		pSPI_Handle->RxLen--;
		pSPI_Handle->pRxBuffer++;
	}

	if(! pSPI_Handle->RxLen)
	{
		// Reception complete
		SPI_CloseReception(pSPI_Handle);
		SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPI_Handle)
{
	uint8_t temp;
	// Clear the OVR flag
	if(pSPI_Handle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPI_Handle->pSPIx->DR;
		temp = pSPI_Handle->pSPIx->SR;
	}
	(void)temp; // Prevent unused var warning

	SPI_ApplicationEventCallback(pSPI_Handle, SPI_EVENT_OVR_ERR);
}

void SPI_CloseTransmisson(SPI_Handle_t *pSPI_Handle)
{
	pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE_Pos);
	pSPI_Handle->pTxBuffer = NULL;
	pSPI_Handle->TxLen = 0;
	pSPI_Handle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPI_Handle)
{
	pSPI_Handle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE_Pos);
	pSPI_Handle->pRxBuffer = NULL;
	pSPI_Handle->RxLen = 0;
	pSPI_Handle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_TypeDef *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// Weak implementation
}
