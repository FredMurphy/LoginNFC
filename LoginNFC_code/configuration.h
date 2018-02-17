/*
 * configuration.h
 *
 *  Created on: 2 Feb 2016
 *      Author: 0xFRED
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

void handleCDCDataReceived(void);
uint8_t retInString (char* string);
void clearBuffer(void);
void storePasswordInRAM(char* newPassword);
void storeUidAndPasswordInFlash(tISO14443A_UidSize uidLength, uint8_t* newUid, char newIsWindows, char* newPassword);
void checkStoreNewPasswordAndTagTimer(void);



#endif /* CONFIGURATION_H_ */
