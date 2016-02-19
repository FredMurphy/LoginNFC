/*
 * configuration.h
 *
 *  Created on: 2 Feb 2016
 *      Author: fred2
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <stdint.h>
#include <string.h>
#include "cdc_serial.h"
#include "hal.h"
#include "USB_app/usbConstructs.h"
#include "USB_config/descriptors.h"
#include "nfc.h"

typedef enum
{
	RUNNING,
	PASSWORD_READY_TO_STORE
} configState;



void handleCDCDataReceived(void);
uint8_t retInString (char* string);
void clearBuffer(void);
void storePasswordInRAM(char* newPassword);
void storeUidAndPasswordInFlash(tISO14443A_UidSize uidLength, uint8_t* newUid, char* newPassword);




#endif /* CONFIGURATION_H_ */
