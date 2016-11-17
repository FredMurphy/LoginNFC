/*
 * cap_touch.c
 *
 *  Created on: 11 Nov 2016
 *      Author: 0xFRED
 */
#include "msp430.h"
#include "cap_touch.h"
#include "gpio_macros.h"

volatile int counter;
int average = 0;
int threshold = 0;

void initCapTouch() {

	    PxDIR(CAP_PORT) |= CAP_OUT;
	    PxDIR(CAP_PORT) &= ~CAP_IN;
	    PxOUT(CAP_PORT) = 0;
	    PxREN(CAP_PORT) = 0;

	    PxIES(CAP_PORT) = 0;			// rising edge
	    PxIFG(CAP_PORT) = 0;			// clear P1 interrupt flag

	    PxIE(CAP_PORT) = CAP_IN;		// enable P1 interrupts
		SFRIE1 |= WDTIE; 	// Enable WDT interrupt

	    // Kick it off
	    counter = 0;
	    PxOUT(CAP_PORT) = CAP_OUT;

}

void captureCapTouchMeasurement()
{
	int measurement = counter;
		counter = 0;

		if (average == 0) {
			average = measurement;
		} else 	{
			// Rolling average and threshold
			if (measurement < average)
				// Only decrease slowly, so we don't get "touch-blind"
				average --;
			else
				average = ((average * 7) + measurement)/8;
		}

		// Fairly low threshold
		threshold = average - average/32;

		touched = (measurement < threshold);
}

bool touchDetected() {
	return touched;
}

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
	// Rising edge?
	int flag = PxIN(CAP_PORT) & CAP_IN;
	// Clear interrupt flag
	P1IFG = 0;

	if (flag)
	{
		PxIES(CAP_PORT) = CAP_IN;	// Wait for falling edge
		PxOUT(CAP_PORT) &= ~CAP_OUT;		// and set capacitor to discharge
	} else {
		PxIES(CAP_PORT) = 0;		// Wait for rising edge
		PxOUT(CAP_PORT) |= CAP_OUT;	// and set capacitor to charge
	}
	counter++;
}


