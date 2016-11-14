/*
 * keyboardplus.c
 *
 *  Created on: 28 Oct 2014
 *      Author: 0xFRED
 */
//#include "USB_API/USB_Common/usb.h"                  // USB-specific functions
#include <stdint.h>
#include <stdlib.h>
#include "USB_API/USB_HID_API/UsbHid.h"
#include "USB_config/descriptors.h"
#include "USB_app/keyboard.h"
#include "keyboard_login.h"

extern volatile uint8_t keySendComplete;
KeyUnion ctrlAltDel;

void initKeyReports() {

	uint8_t i;

	for (i=0; i<8; i++) {
		ctrlAltDel.keyArray[i] = 0;
	}

	ctrlAltDel.keyReport.modifiers = MODIFIER_LEFT_CTRL | MODIFIER_LEFT_ALT;

	ctrlAltDel.keyReport.keys[0] = KEY_DELETE - 136;
	for (i=1; i<6; i++) {
		ctrlAltDel.keyReport.keys[i] = 0;
	}
}

void sendReport(KeyUnion report)
{
	USBHID_sendReport(report.keyArray, HID0_INTFNUM);
}

void sendLockPC() {

    keySendComplete = FALSE;
    Keyboard_press(KEY_LEFT_GUI);
    while(!keySendComplete);

    keySendComplete = FALSE;
    Keyboard_press('l');
	while(!keySendComplete);


    Keyboard_releaseAll();
    while(!keySendComplete);
    keySendComplete = FALSE;

}

void sendCtrlAltDel() {

	keySendComplete = 0;
	sendReport(ctrlAltDel);
	while(!keySendComplete);

	keySendComplete = 0;
	Keyboard_releaseAll();
	while(!keySendComplete);

}

void sendReturn() {

	keySendComplete = 0;
	Keyboard_press(KEY_RETURN);
	while(!keySendComplete);

	keySendComplete = 0;
	Keyboard_release(KEY_RETURN);
	while(!keySendComplete);

}



