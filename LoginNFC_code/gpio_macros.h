#ifndef GPIO_MACROS_H_
#define GPIO_MACROS_H_

/*
 * Usage:
 * 	#define LED_PORT 1
 * 	#define LED_PIN  BIT1
 *
 * 	PxOUT(LED_PORT) ^= LED_PIN;
 */

// redirection to macros below allow prt to be expanded rather than taken literally
#define PxIN(port)	PzIN(port)
#define PxOUT(port) PzOUT(port)
#define PxDIR(port) PzDIR(port)
#define PxSEL(port)	PzSEL(port)
#define PxREN(port) PzREN(port)
#define PxIE(port)  PzIE(port)
#define PxIES(port) PzIES(port)
#define PxIFG(port) PzIFG(port)

#define PzIN(port)  P##port##IN
#define PzOUT(port) P##port##OUT
#define PzDIR(port) P##port##DIR
#define PzSEL(port) P##port##SEL
#define PzREN(port) P##port##REN
#define PzIE(port)  P##port##IE
#define PzIES(port) P##port##IES
#define PzIFG(port) P##port##IFG

#endif
