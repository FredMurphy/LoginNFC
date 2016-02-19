
#include <string.h>
#include "driverlib.h"

#include "USB_config/descriptors.h"
#include "USB_API/USB_Common/device.h"
#include "USB_API/USB_Common/usb.h"                     //USB-specific functions
#include "USB_API/USB_CDC_API/UsbCdc.h"
#include "USB_app/usbConstructs.h"


unsigned int MAX_STR_LENGTH = 256;

cdcSend(char* message) {

    // Send the response over USB
    uint8_t retCode = cdcSendDataInBackground((uint8_t*)message, strlen(message), CDC0_INTFNUM, 10);
    if (retCode) {
    	__no_operation();
    }
}




