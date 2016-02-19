/*
 * File Name: spi.c
 *
 * Description: Contains functions to initialize SPI interface using
 * USCI_B0 and communicate with the TRF797x via this interface.
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "spi.h"
#include "trf797x.h"

//===============================================================

void SpiUsciSet(void);

//===============================================================
// NAME: void SpiDirectCommand (uint8_t *pui8Buffer)
//
// BRIEF: Is used in SPI mode to transmit a Direct Command to
// reader chip.
//
// INPUTS:
//	Parameters:
//		uint8_t		*pui8Buffer		Direct Command
//
// OUTPUTS:
//
// PROCESS:	[1] transmit Direct Command
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================

void 
SpiDirectCommand(uint8_t ui8Command)
{	
	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// set Address/Command Word Bit Distribution to command
	ui8Command = (0x80 | ui8Command);					// command
	ui8Command = (0x9f & ui8Command);					// command code

	SpiSendByte(ui8Command);

	SLAVE_SELECT_HIGH; 						//Stop SPI Mode
}

//===============================================================
// NAME: void SpiDirectMode (void)
//
// BRIEF: Is used in SPI mode to start Direct Mode.
//
// INPUTS:
//
// OUTPUTS:
//
// PROCESS:	[1] start Direct Mode
//
// NOTE: No stop condition
//
//===============================================================

void
SpiDirectMode(void)
{		
	uint8_t pui8Command[2];

	pui8Command[0] = TRF797x_STATUS_CONTROL;
	pui8Command[1] = TRF797x_STATUS_CONTROL;
	SpiReadSingle(&pui8Command[1]);
	pui8Command[1] |= 0x60;						// RF on and BIT 6 in Chip Status Control Register set
	SpiWriteSingle(pui8Command);
}  	

//===============================================================
// NAME: void SpiRawWrite (uint8_t *pui8Buffer, uint8_t length)
//
// BRIEF: Is used in SPI mode to write direct to the reader chip.
//
// INPUTS:
//	Parameters:
//		uint8_t		*pui8Buffer		raw data
//		uint8_t		length		number of data bytes
//
// OUTPUTS:
//
// PROCESS:	[1] send raw data to reader chip
//
//===============================================================

void
SpiRawWrite(uint8_t * pui8Buffer, uint8_t ui8Length)
{
//	uint8_t ui8TempVar = 0;

	//Start SPI Mode
	SLAVE_SELECT_LOW;

	while(ui8Length-- > 0)
	{
		// Check if USCI_B0 TX buffer is ready
		while (!(SPI_TX_READY));

		// Transmit data
		SPI_TX_BUF = *pui8Buffer;

		while(SPI_TX_BUSY);	// Wait while SPI state machine is busy

//		ui8TempVar=SPI_RX_BUF;

		pui8Buffer++;
	}

	// Stop SPI Mode
	SLAVE_SELECT_HIGH;
}
//===============================================================
// NAME: void SpiReadCont (uint8_t *pui8Buffer, uint8_t length)
//
// BRIEF: Is used in SPI mode to read a specified number of
// reader chip registers from a specified address upwards.
//
// INPUTS:
//	Parameters:
//		uint8_t		*pui8Buffer		address of first register
//		uint8_t		length		number of registers
//
// OUTPUTS:
//
// PROCESS:	[1] read registers
//			[2] write contents to *pui8Buffer
//
//===============================================================

void
SpiReadCont(uint8_t * pui8Buffer, uint8_t ui8Length)
{	
	SLAVE_SELECT_LOW; 							//Start SPI Mode

	// Address/Command Word Bit Distribution
	*pui8Buffer = (0x60 | *pui8Buffer); 					// address, read, continuous
	*pui8Buffer = (0x7f &*pui8Buffer);						// register address

	SpiSendByte(*pui8Buffer);

	while(ui8Length-- > 0)
	{
		*pui8Buffer = SpiReceiveByte();
		pui8Buffer++;
	}
	while(SPI_TX_BUSY)
	{
	}
	SLAVE_SELECT_HIGH; 						// Stop SPI Mode
}

//===============================================================
// NAME: void SpiReadSingle (uint8_t *pui8Buffer)
//
// BRIEF: Is used in SPI mode to read specified reader chip
// registers.
//
// INPUTS:
//	Parameters:
//		uint8_t		*pui8Buffer		addresses of the registers
//
// OUTPUTS:
//
// PROCESS:	[1] read registers
//			[2] write contents to *pui8Buffer
//
//===============================================================

void
SpiReadSingle(uint8_t * pui8Buffer)
{			
	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// Address/Command Word Bit Distribution
	*pui8Buffer = (0x40 | *pui8Buffer); 			// address, read, single
	*pui8Buffer = (0x5f & *pui8Buffer);				// register address

	SpiSendByte(*pui8Buffer);					// Previous data to TX, RX

	*pui8Buffer = SpiReceiveByte();

	SLAVE_SELECT_HIGH; 						// Stop SPI Mode
}

uint8_t SpiReceiveByte(void)
{
	SPI_TX_BUF = 0x00;

	while (SPI_TX_BUSY);

	return SPI_RX_BUF;
}

void SpiSendByte(uint8_t ui8TxByte)
{
	SPI_TX_BUF = ui8TxByte;

	while (SPI_TX_BUSY);
}

//===============================================================
// Settings for SPI Mode                                        ;
// 02DEC2010	RP	Original Code
//===============================================================

void
SpiSetup(void)
{	
	TRF_ENABLE_SET;

	IRQ_PIN_SET;
	IRQ_EDGE_SET;								// rising edge interrupt

	SpiUsciSet();								// Set the USART

}

//===============================================================
// NAME: void SpiUsciExtClkSet (void)
//
// BRIEF: Is used to switch SPI data clock from DCO to more
// stabile extern clock
//
// INPUTS:
//
// OUTPUTS:
//
// PROCESS:	[1] switch SPI data clock
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	changed SPI clock from 6.78MHz to 1.70MHz
//===============================================================

void
SpiUsciExtClkSet(void)	  							//Uses USCI_B0
{
	SPI_CTL1 |= UCSWRST;                     		// Enable SW reset
	SPI_CTL0 |= UCCKPH + UCMSB + UCMST + UCSYNC;  	// 3-pin, 8-bit SPI master
	SPI_CTL0 &= ~UCCKPH;

	SPI_CTL1 |= UCSSEL_2;                     		// SMCLK

	SPI_BR0 = 0x04;									// 1.70 MHz
	SPI_BR1 = 0;

    SPI_PORT_FUNC1 |= SPI_CLK + SPI_MISO + SPI_MOSI;
    SPI_PORT_FUNC2 |= SPI_CLK + SPI_MISO + SPI_MOSI;

	SLAVE_SELECT_PORT_SET;  						// P3.0 - Slave Select
	SLAVE_SELECT_HIGH;     							// Slave Select - inactive ( high)

	SPI_CTL1 &= ~UCSWRST;                     		// **Initialize USCI state machine**
}

//===============================================================
// NAME: void SpiUsciSet (void)
//
// BRIEF: Is used to set USCI B0 for SPI communication
//
// INPUTS:
//
// OUTPUTS:
//
// PROCESS:	[1] make settings
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	reduced SPI clock frequency
//===============================================================

void
SpiUsciSet(void)									//Uses USCI_B0
{
	SPI_CTL1 |= UCSWRST;							// Enable SW reset
	SPI_CTL0 |= UCCKPH + UCMSB + UCMST + UCSYNC;	// 3-pin, 8-bit SPI master
	SPI_CTL0 &= ~UCCKPH;
	SPI_CTL1 |= UCSSEL_2;							// SMCLK

	SPI_BR0 = 0x04;
	SPI_BR1 = 0;
	SPI_PORT_FUNC1 |= SPI_CLK + SPI_MISO + SPI_MOSI;
	SPI_PORT_FUNC2 |= SPI_CLK + SPI_MISO + SPI_MOSI;

	SLAVE_SELECT_PORT_SET;							// P2.1 - Slave Select
	SLAVE_SELECT_HIGH;								// Slave Select - inactive ( high)

	SPI_CTL1 &= ~UCSWRST;							// **Initialize USCI state machine**
}


//===============================================================
// NAME: void SpiWriteCont (uint8_t *pui8Buffer, uint8_t length)
//
// BRIEF: Is used in SPI mode to write to a specific number of
// reader chip registers from a specific address upwards.
//
// INPUTS:
//	uint8_t	*pui8Buffer	address of first register followed by the
//					contents to write
//	uint8_t	length	number of registers + 1
//
// OUTPUTS:
//
// PROCESS:	[1] write to the registers
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================

void
SpiWriteCont(uint8_t * pui8Buffer, uint8_t ui8Length)
{	
	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// Address/Command Wort Bit Distribution
	*pui8Buffer = (0x20 | *pui8Buffer); 				// address, write, continuous
	*pui8Buffer = (0x3f & *pui8Buffer);					// register address

	while(ui8Length-- > 0)
	{	
		SpiSendByte(*pui8Buffer++);
	}

	SLAVE_SELECT_HIGH; 						// Stop SPI Mode
}

//===============================================================
// NAME: void SpiWriteSingle (uint8_t *pui8Buffer, uint8_t length)
//
// BRIEF: Is used in SPI mode to write to a specified reader chip
// registers.
//
// INPUTS:
//	uint8_t	*pui8Buffer	addresses of the registers followed by the
//					contends to write
//	uint8_t	length	number of registers * 2
//
// OUTPUTS:
//
// PROCESS:	[1] write to the registers
//
// CHANGE:
// DATE  		WHO	DETAIL
// 24Nov2010	RP	Original Code
// 07Dec2010	RP	integrated wait while busy loops
//===============================================================

void
SpiWriteSingle(uint8_t * pui8Buffer)
{
	SLAVE_SELECT_LOW; 						// Start SPI Mode

	// Address/Command Word Bit Distribution
	// address, write, single (fist 3 bits = 0)
	*pui8Buffer = (0x1f & *pui8Buffer);				// register address


	SpiSendByte(*pui8Buffer++);
	SpiSendByte(*pui8Buffer++);

	SLAVE_SELECT_HIGH; 						// Stop SPI Mode
}
