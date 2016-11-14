/*
 * configuration.c
 *
 *  Created on: 2 Feb 2016
 *      Author: fred2
 */
#include "configuration.h"
#include "nfc.h"

#define MAX_STR_LENGTH 64
char wholeString[MAX_STR_LENGTH] = ""; // Entire input str from last 'return'
char tempPassword[MAX_STR_LENGTH];

#define MAX_PASSWORD_LENGTH 64
#define MAX_UID_LENGTH ISO14443A_UID_TRIPLE

char tempPassword[];

// Store in info D (0x1800)
// #pragma DATA_SECTION(uidLength,".infoD")
// Not sure how to locate a whole block of variables sequentially
#pragma location=0x1800
tISO14443A_UidSize uidLength = ISO14443A_UID_UNKNOWN;
#pragma location=0x1802
char uid[MAX_UID_LENGTH] = "\0";
#pragma location=0x180C
char password[MAX_PASSWORD_LENGTH] = "password\0";


void handleCDCDataReceived(void) {

    // Holds the new addition to the string
    char pieceOfString[MAX_STR_LENGTH] = "";

    // Add bytes in USB buffer to the string
    cdcReceiveDataInBuffer((uint8_t*)pieceOfString, MAX_STR_LENGTH, CDC0_INTFNUM); // Get the next piece of the string

    // Append new piece to the whole
    strcat(wholeString,pieceOfString);

    // Echo back the characters received
    cdcSend(pieceOfString);

    // Has the user pressed return yet?
    if (retInString(wholeString)){

    	switch(mode) {

    		case PASSWORD_READY_TO_STORE:
    			cdcSend("Cancelled.\r\n");
    			clearBuffer();
				LED_OFF;
				setModeTouch();
				break;

    		default:
    			storePasswordInRAM(wholeString);
    			cdcSend("\r\nScan NFC tag to store new password...\r\n");
    			LED_RED;
    			LED_YELLOW;
    			setModePassword();
    			break;
    	}

    }

}

void checkStoreNewPasswordAndTagTimer() {
	if (mode == PASSWORD_READY_TO_STORE) {
		/*
		if (--readTagToStoreDelay == 0) {
			cdcSend("Timed out.\r\n");
			setModeTouch();
		}
		*/
	}
}

void clearBuffer() {
	uint16_t i;
	for (i = 0; i < MAX_STR_LENGTH; i++)
		wholeString[i] = 0x00;
}

void storePasswordInRAM(char* newPassword) {

	strcpy(tempPassword, newPassword);

	// Clear the string in preparation for the next one
	clearBuffer();
}

void storeUidAndPasswordInFlash(tISO14443A_UidSize newUidLength, uint8_t* newUid, char* newPassword) {

	  FCTL3 = FWKEY;                            // Clear Lock bit
	  FCTL1 = FWKEY+ERASE;                      // Set Erase bit
	  *password = 0;                           	// Dummy write to erase Flash seg
	  FCTL1 = FWKEY+WRT;                        // Set WRT bit for write operation

	  // UID
	  uidLength = newUidLength;
	  uint16_t i;
	  for (i = 0; i < MAX_UID_LENGTH; i++)
		  uid[i] = newUid[i];

	  // Password
	  strcpy(password, newPassword);

	  FCTL1 = FWKEY;                            // Clear WRT bit
	  FCTL3 = FWKEY+LOCK;                       // Set LOCK bit

	  cdcSend("New password stored.\r\n");

	  LED_OFF;
	  setModeTouch();
}

/*
 * ======== retInString ========
 */
// This looks like it is too complex!
// This function returns true if there's an 0x0D character in the string; and if
// so, it trims the 0x0D and anything that had followed it.
uint8_t retInString (char* string)
{
    uint8_t retPos = 0,i,len;
    char tempStr[MAX_STR_LENGTH] = "";

    strncpy(tempStr,string,strlen(string));  // Make a copy of the string
    len = strlen(tempStr);

    // Find 0x0D; if not found, retPos ends up at len
    while ((tempStr[retPos] != 0x0A) && (tempStr[retPos] != 0x0D) &&
           (retPos++ < len)) ;

    // If 0x0D was actually found...
    if ((retPos < len) && (tempStr[retPos] == 0x0D)){
        for (i = 0; i < MAX_STR_LENGTH; i++){ // Empty the buffer
            string[i] = 0x00;
        }

        //...trim the input string to just before 0x0D
        strncpy(string,tempStr,retPos);

        //...and tell the calling function that we did so
        return (TRUE) ;

    // If 0x0D was actually found...
    } else if ((retPos < len) && (tempStr[retPos] == 0x0A)){
        // Empty the buffer
        for (i = 0; i < MAX_STR_LENGTH; i++){
            string[i] = 0x00;
        }

        //...trim the input string to just before 0x0D
        strncpy(string,tempStr,retPos);

        //...and tell the calling function that we did so
        return (TRUE) ;
    } else if (tempStr[retPos] == 0x0D){
        for (i = 0; i < MAX_STR_LENGTH; i++){  // Empty the buffer
            string[i] = 0x00;
        }
        // ...trim the input string to just before 0x0D
        strncpy(string,tempStr,retPos);
        // ...and tell the calling function that we did so
        return (TRUE) ;
    } else if (retPos < len){
        for (i = 0; i < MAX_STR_LENGTH; i++){  // Empty the buffer
            string[i] = 0x00;
        }

        //...trim the input string to just before 0x0D
        strncpy(string,tempStr,retPos);

        //...and tell the calling function that we did so
        return (TRUE) ;
    }

    return (FALSE) ; // Otherwise, it wasn't found
}
