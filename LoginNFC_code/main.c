#include <string.h>

#include "driverlib.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                  // USB-specific functions
#include "USB_API/USB_HID_API/UsbHid.h"
#include "USB_app/keyboard.h"
#include "USB_app/usbConstructs.h"

#include "keyboard_login.h"
#include "hal.h"
#include "nfc.h"
#include "configuration.h"
#include "cap_touch.h"

extern char tempIsWindows;
extern char tempPassword[];
extern tISO14443A_UidSize uidLength;
extern char uid[];
extern char isWindows;
extern char password[];

tISO14443A_UidSize size;
uint8_t *guid;

// Serial comms
volatile uint8_t cdcDataReceived;
volatile uint8_t keySendComplete = TRUE;

void unlockPC(void);
void waitForUsbActive(void);

/*
 * ======== main ========
 */
void main (void)
{
    // Minumum Vcore setting required for the USB API is PMM_CORE_LEVEL_2 .
    PMM_setVCore(PMM_CORE_LEVEL_2);
    initPorts();           // Config GPIOS for low-power (output low)
    initClocks(8000000);   // Config clocks. MCLK=SMCLK=FLL=8MHz; ACLK=REFO/16=2kHz

    initCapTouch();

    Keyboard_init();       // Init keyboard report
    USB_setup(TRUE, TRUE); // Init USB & events; if a host is present, connect

    Keyboard_init();       // Init keyboard report
    initKeyReports();

    __enable_interrupt();  // Enable global interrupts

	McuDelayMillisecond(10);

	// Set the SPI SS high
	SLAVE_SELECT_PORT_SET;
	SLAVE_SELECT_HIGH;

	// Set TRF Enable Pin high
	TRF_ENABLE_SET;
	TRF_ENABLE;

	// Wait until TRF system clock started
	McuDelayMillisecond(5);

	// Initialize SPI settings for communication with TRF
	Trf797xCommunicationSetup();

	// LEDs
	LED_PORT_SET;

	clearBuffer();

	// Set up TRF initial settings
	Trf797xInitialSettings();

	startTimer();

	// Enable global interrupts
	__bis_SR_register(GIE);


	// Wait for USB to be up and stable
	waitForUsbActive();


	while(1)
	{

		if (mode == SCAN_FOR_NFC || mode == PASSWORD_SCAN_NFC) {

			if (Nfc_FindTag() == STATUS_SUCCESS)
			{
				size = Iso14443a_Get_UidSize();
				guid = Iso14443a_Get_Uid();

				if (mode == PASSWORD_SCAN_NFC) {
					setModePause();
					storeUidAndPasswordInFlash(size, guid, tempIsWindows, tempPassword);
					mode = PAUSE;

				} else {
					// Compare and unlock
					if ((size == uidLength) && (memcmp(guid, uid, uidLength) == 0))
					{
						setModePause();
						unlockPC();
					}
				}
			}
		}

        // If true, some data is in the buffer; begin receiving a cmd
        if (cdcDataReceived){
        	cdcDataReceived = FALSE;
        	handleCDCDataReceived();
        }

	}

}


void unlockPC(void) {

    size_t passwordLength = strlen(password);

    if (isWindows > 0) {
    	sendCtrlAltDel();
    	__delay_cycles(8000000);
    }

    uint8_t i;
    for (i=0; i<passwordLength; i++) {

        keySendComplete = FALSE;
    	Keyboard_write(password[i]);
        while(!keySendComplete);

        keySendComplete = FALSE;
        Keyboard_releaseAll();
        while(!keySendComplete);

    }

   	sendReturn();

}



void waitForUsbActive(void) {

	LED_YELLOW;

    while (1)
    {

        switch(USB_getConnectionState())
        {
            // This case is executed while your device is enumerated on the
            // USB host
            case ST_ENUM_ACTIVE:

            	LED_OFF;
            	return;

            // These cases are executed while your device is disconnected from
            // the host (meaning, not enumerated); enumerated but suspended
            // by the host, or connected to a powered hub without a USB host
            // present.
            case ST_PHYS_DISCONNECTED:
            case ST_ENUM_SUSPENDED:
            case ST_PHYS_CONNECTED_NOENUM_SUSP:
                __bis_SR_register(LPM3_bits + GIE);
                _NOP();
                break;

            // The default is executed for the momentary state
            // ST_ENUM_IN_PROGRESS.  Usually, this state only last a few
            // seconds.  Be sure not to enter LPM3 in this state; USB
            // communication is taking place here, and therefore the mode must
            // be LPM0 or active-CPU.
            case ST_ENUM_IN_PROGRESS:
            default:;
        }
    }  //while(1)

}

/*
 * ======== UNMI_ISR ========
 */
#pragma vector = UNMI_VECTOR
__interrupt void UNMI_ISR (void)
{
    switch (__even_in_range(SYSUNIV, SYSUNIV_BUSIFG))
    {
        case SYSUNIV_NONE:
            __no_operation();
            break;
        case SYSUNIV_NMIIFG:
            __no_operation();
            break;
        case SYSUNIV_OFIFG:
            UCS_clearFaultFlag(UCS_XT2OFFG);
            UCS_clearFaultFlag(UCS_DCOFFG);
            SFR_clearInterrupt(SFR_OSCILLATOR_FAULT_INTERRUPT);
            break;
        case SYSUNIV_ACCVIFG:
            __no_operation();
            break;
        case SYSUNIV_BUSIFG:
            // If the CPU accesses USB memory while the USB module is
            // suspended, a "bus error" can occur.  This generates an NMI.  If
            // USB is automatically disconnecting in your software, set a
            // breakpoint here and see if execution hits it.  See the
            // Programmer's Guide for more information.
            SYSBERRIV = 0; //clear bus error flag
            USB_disable(); //Disable
    }
}

//Released_Version_5_00_01
