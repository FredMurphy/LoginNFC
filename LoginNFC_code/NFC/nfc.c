/*
 * File Name: nfc.c
 *
 * Description: Contains functions to handle processing the NFC/RFID
 * stack.
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

#include "nfc.h"

//===============================================================
// NAME:
//
// BRIEF:
//
// INPUTS:
//
// OUTPUTS: None
//
// PROCESS:
//
//===============================================================

uint8_t Nfc_FindTag(void)
{
	// Test - flash to show how often we scan
	// LED_RED_PORT ^= LED_RED_PIN;

	// Clear IRQ Flags before enabling TRF7970A
	IRQ_CLR;
	IRQ_ON;
	TRF_ENABLE;

	// Must wait at least 4.8 mSec to allow TRF7970A to initialize.
	McuDelayMillisecond(5);

	uint8_t status = Nfc_Iso14443a_App();

	// Short delay before reading next tag
	// McuDelayMillisecond(100);

	IRQ_OFF;
	TRF_DISABLE;

	return status;
}

//===============================================================
//
// Nfc_Iso14443a_App - Customizeable application to search
// for ISO14443A/NFC Type 2/4A Tag Platform compliant tags.
//
// This function configures TRF797x for ISO14443A, turns on RF
// field, and waits for a guard time to allow PICC to power up.
//
// Tag detection is handled in this example by issuing a REQA
// command, and if a reply is received, the ISO14443A anti-
// collision process is run until a single tag has provided it's
// entire UID. If the tag is Type 4 compliant, an NDEF read
// function is called. If the tag is not Type 4 Compliant, then
// a Type 2 Application is called.
//
// \return None.
//
//===============================================================

tISO14443A_UidSize size;
uint8_t *guid;

uint8_t Nfc_Iso14443a_App(void)
{

	Trf797xTurnRfOn();						// Ensure TRF797x is outputting an RF Field

	Trf797xWriteInitiatorSetup(0x88);		// Configure the TRF797x for ISO14443A @ 106kbps and Receive no CRC

	// When a PICC is exposed to an unmodulated operating field
	// it shall be able to accept a quest within 5 ms.
	// PCDs should periodically present an unmodulated field of at least
	// 5.1 ms duration. (ISO14443-3)
	McuDelayMillisecond(6);

	Iso14443a_Set_RecursionCount(0); 		// Clear the recursion count for anticollision loops

	P6OUT = 0x01;
	P6OUT = 0x00;

	uint8_t status = Iso14443a_TagSelection(REQA);
	if (status == STATUS_SUCCESS)	//  Do a complete anticollision sequence as described in ISO14443-3 standard for type A
	{
		size = Iso14443a_Get_UidSize();
		guid = Iso14443a_Get_Uid();
	}

	Trf797xTurnRfOff();

	return status;
}

