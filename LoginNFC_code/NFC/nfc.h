/*
 * File Name: nfc.h
 *
 * Description: Header file for all functions for nfc.c
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

#ifndef NFC_H_
#define NFC_H_

//================================================================

#include "iso14443a.h"
//#include "iso14443b.h"
//#include "iso15693.h"
//#include "felica.h"

//================================================================

#define RFID_READER_FW_VERSION		"1.17"
#define RFID_READER_FW_DATE			"Dec 16th, 2015"

#define ENABLE_14443A
//#define ENABLE_14443B
//#define ENABLE_15693
//#define ENABLE_FELICA

//================================================================

uint8_t Nfc_FindTag(void);

uint8_t Nfc_Iso14443a_App(void);
void Nfc_Iso14443b_NdefApp(void);
void Nfc_FeliCa_App(void);
void Nfc_Iso15693_App(void);

void Nfc_Iso14443a_Type4NdefApp(void);
void Nfc_Iso14443a_Type2App(uint8_t ui8ReadBlocks);

void Nfc_Iso15693_ReadTag(uint8_t ui8ReqFlag);
void Nfc_Iso15693_ReadExtendedTag(uint8_t ui8ReqFlag);

#endif /* NFC_H_ */
