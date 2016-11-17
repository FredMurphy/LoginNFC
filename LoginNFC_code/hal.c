/*
 * ======== hal.c ========
 */
#include "msp430.h"
#include "driverlib.h"
#include "hal.h"
#include "trf797x.h"
#include "ucs.h"
#include "cap_touch.h"

#define GPIO_ALL	GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3| \
					GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7

/*
 * This function drives all the I/O's as output-low, to avoid floating inputs
 * (which cause extra power to be consumed).  This setting is compatible with  
 * TI FET target boards, the F5529 Launchpad, and F5529 Experimenters Board;  
 * but may not be compatible with custom hardware, which may have components  
 * attached to the I/Os that could be affected by these settings.  So if using
 * other boards, this function may need to be modified.
 */
void initPorts(void)
{
#ifdef __MSP430_HAS_PORT1_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT2_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT3_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT4_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT5_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT6_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT7_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT8_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORT9_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_P9, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_P9, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORTA_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_PA, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_PA, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORTB_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_PB, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_PB, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORTC_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_PC, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_PC, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORTD_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_PD, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_PD, GPIO_ALL);
#endif

#ifdef __MSP430_HAS_PORTJ_R__
    GPIO_setOutputLowOnPin(GPIO_PORT_PJ, GPIO_ALL);
    GPIO_setAsOutputPin(GPIO_PORT_PJ, GPIO_ALL);
#endif
}

/* Configures the system clocks:
* MCLK = SMCLK = DCO/FLL = mclkFreq (expected to be expressed in Hz)
* ACLK = FLLref = REFO/16=32kHz
*
* XT2 is not configured here.  Instead, the USB API automatically starts XT2
* when beginning USB communication, and optionally disables it during USB
* suspend.  It's left running after the USB host is disconnected, at which
* point you're free to disable it.  You need to configure the XT2 frequency
* in the Descriptor Tool (currently set to 4MHz in this example).
* See the Programmer's Guide for more information.
*/
void initClocks(uint32_t mclkFreq)
{
	UCS_initClockSignal(
	   UCS_FLLREF,
	   UCS_REFOCLK_SELECT,
	   UCS_CLOCK_DIVIDER_1);

	// ACLK = 32/16 = 2kHz
	UCS_initClockSignal(
	   UCS_ACLK,
	   UCS_REFOCLK_SELECT,
	   UCS_CLOCK_DIVIDER_16);

    UCS_initFLLSettle(
        mclkFreq/1000,
        mclkFreq/32768);
}

void McuCounterSet(void) {
#if (defined (__MSP430F5529__) || defined (__MSP430F5510__))
	TA1CTL |= TASSEL_1 + TACLR;      // ACLK = 1 kHz?, timer stopped
	TA1CCTL0 |= CCIE;                // compare interrupt enable

	TA0CTL |= TASSEL_1 + TACLR;      // ACLK = 1 kHz?, timer stopped
	TA0CCTL0 |= CCIE;                // compare interrupt enable

#endif
}

//===========================================================================
//
// McuDelayMillisecond - Delay for inputted number of millisecond
//
// \param n_ms is the number of milliseconds to delay by
//
// Function to delay is approximately one millisecond, looping until the
// inputted number of milliseconds have gone by. DELAY_1ms must be
// calibrated based on the clock speed of the MCU
//
// \return None.
//
//===========================================================================

void McuDelayMillisecond(uint32_t n_ms) {
	while (n_ms--) {
		__delay_cycles(DELAY_1ms);		// clock speed in Hz divined by 1000
	}
}

//extern volatile uint8_t delayUntilReadNfc;

void startTimer() {

	// Use watchdog as timer
	WDTCTL = WDT_MDLY_32;                     // WDT 32ms, SMCLK, interval timer

	mode = WAIT_FOR_TOUCH;

	/*
	// Using TimerB because A1 seemed to not run continuously
	TBCCTL0 = CCIE;                           // TBCCR0 interrupt enabled
	TBCCR0 = 750 * 2;	// About 750ms?
	TBCTL = TBSSEL_1 + MC_1 + TBCLR;          // SMCLK, upmode, clear TBR
	*/
}

#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
{
	captureCapTouchMeasurement();

	switch (mode)
	{
	case WAIT_FOR_TOUCH:
		if (touchDetected()) {
			setModeNFC();
		}
		break;

	case SCAN_FOR_NFC:
	case PASSWORD_READY_TO_STORE:
	case PAUSE:
		if (cyclesRemaining-- <= 0) {
			setModeTouch();
		}
		break;

	default:
		mode = WAIT_FOR_TOUCH;
		LED_OFF;
		break;
	}
}

void setModeTouch(void) {
	mode = WAIT_FOR_TOUCH;
	LED_OFF;
	cyclesRemaining = 500;
}

void setModeNFC(void) {
	mode = SCAN_FOR_NFC;
	LED_YELLOW;
	cyclesRemaining = 1000;
}
void setModePassword(void)
{
	mode = PASSWORD_READY_TO_STORE;
	LED_YELLOW;
	cyclesRemaining = 2500;
}
void setModePause() {
	mode = PAUSE;
	LED_GREEN;
	cyclesRemaining = 500;
}

/*
// NFC timer ISR
#pragma vector=TIMERB0_VECTOR
__interrupt void TimerA1Handler(void) {

	// Decrement the delay (normally 1, but increased after a login)
	if (delayUntilReadNfc)
		delayUntilReadNfc--;

}
*/
