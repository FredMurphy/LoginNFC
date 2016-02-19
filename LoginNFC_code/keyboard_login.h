
#ifndef __KEYBOARD_LOGIN__
#define __KEYBOARD_LOGIN__

#include "USB_app/keyboard.h"

#define MODIFIER_LEFT_CTRL		0x01
#define MODIFIER_LEFT_SHIFT		0x02
#define MODIFIER_LEFT_ALT		0x04
#define MODIFIER_LEFT_GUI   	0x08
#define MODIFIER_RIGHT_CTRL 	0x10
#define MODIFIER_RIGHT_SHIFT	0x20
#define MODIFIER_RIGHT_ALT		0x40
#define MODIFIER_RIGHT_GUI		0x80

void initKeyReports();
void sendReport(KeyUnion report);
void sendLockPC();
void sendCtrlAltDel();
void sendReturn();

#endif /* if defined(__KEYBOARD_LOGIN__) */
