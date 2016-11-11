/*
 * cap_touch.c
 *
 *  Created on: 11 Nov 2016
 *      Author: fred2
 */
#include "msp430.h"
#include "cap_touch.h"
#include "gpio_macros.h"

volatile int counter;
int average = 0;
int threshold = 0;

void initCapTouch() {

		WDTCTL = WDT_MDLY_32;                     // WDT 32ms, SMCLK, interval timer

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

#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	volatile int measurement = counter;
	counter = 0;

	if (measurement < threshold) {
		// Touch!
		P1OUT |= LED;
		return;
	} else {
		P1OUT &= ~LED;
	}

	if (average == 0) {
		average = measurement;
	} else 	{
		// Rolling average and threshold
		average = ((average * 7) + measurement)/8;
	}
	threshold = average - average/16;
}

