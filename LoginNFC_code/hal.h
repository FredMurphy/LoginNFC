/*
 * ======== hal.h ========
 */

#ifndef HAL_H_
#define HAL_H_

#include "MSP430.h" 		// Processor specific header
#include "types.h"


void initPorts(void);
void initClocks(uint32_t mclkFreq);
void McuCounterSet(void);
void McuDelayMillisecond(uint32_t n_ms);
void startNfcTimer();

//=====MCU constants=============================================
#if defined (__MSP430F5529__)
#define LED_PORT_SET	P1DIR |= LED_RED_PIN; P4DIR |= LED_GREEN_PIN
#define LED_RED_PORT	P1OUT
#define LED_RED_PIN 	BIT0
#define LED_GREEN_PORT	P4OUT
#define LED_GREEN_PIN	BIT7
#define LED_OFF			LED_RED_PORT &= ~LED_RED_PIN; LED_GREEN_PORT &= ~LED_GREEN_PIN
#define LED_RED			LED_RED_PORT |= LED_RED_PIN; LED_GREEN_PORT &= ~LED_GREEN_PIN
#define LED_GREEN		LED_RED_PORT &= ~LED_RED_PIN; LED_GREEN_PORT |= LED_GREEN_PIN
#define LED_YELLOW		LED_RED_PORT |= LED_RED_PIN; LED_GREEN_PORT |= LED_GREEN_PIN

#define TRF_ENABLE_SET	P4DIR |= BIT1		// P4.1 is switched in output direction
#define	TRF_ENABLE		P4OUT |= BIT1		// EN pin on the TRF7970A
#define TRF_DISABLE		P4OUT &= ~BIT1

// IRQ on 2.7
#define IRQ_PIN_SET		P2DIR &= ~IRQ_PIN;
#define IRQ_PIN         BIT7
#define IRQ_PORT		P2IN
#define IRQ_ON			P2IE |= IRQ_PIN
#define IRQ_OFF			P2IE &= ~IRQ_PIN
#define IRQ_EDGE_SET	P2IES &= ~IRQ_PIN		// Rising edge interrupt
#define IRQ_CLR			P2IFG = 0x00
#define IRQ_REQ_REG		P2IFG


#define SLAVE_SELECT_PORT_SET   P4DIR |= BIT2;
#define SLAVE_SELECT_HIGH       P4OUT |= BIT2;
#define SLAVE_SELECT_LOW        P4OUT &= ~BIT2;

#define SPI_PORT_FUNC1      P3SEL
#define SPI_PORT_FUNC2      P3SEL
#define SPI_DIR             P3DIR
#define SPI_OUT             P3OUT
#define SPI_CLK             BIT2
#define SPI_MISO            BIT1
#define SPI_MOSI            BIT0
#define SPI_CTL0			UCB0CTL0
#define SPI_CTL1			UCB0CTL1
#define SPI_BR0				UCB0BR0
#define SPI_BR1				UCB0BR1
#define SPI_RX_BUF			UCB0RXBUF
#define SPI_TX_BUF			UCB0TXBUF
#define SPI_RX_READY		UCB0IFG & UCRXIFG
#define SPI_TX_READY		UCB0IFG & UCTXIFG
#define SPI_TX_BUSY			UCB0STAT & UCBUSY

//-----Counter-timer constants-----------------------------------

#define COUNTER_VALUE       TA0CCR0                 //counter register
#define START_COUNTER       TA0CTL |=  MC1        	//start counter in up mode
#define STOP_COUNTER        TA0CTL &= ~(MC0 + MC1)  //stops the counter
#define RESET_COUNTER       TA0CTL |= TACLR	    	//Resets and stops counter.

#endif

#if defined (__MSP430F5510__)
#define LED_PORT_SET	P1DIR |= LED_RED_PIN + LED_GREEN_PIN
#define LED_RED_PORT	P1OUT
#define LED_RED_PIN 	BIT1
#define LED_GREEN_PORT	P1OUT
#define LED_GREEN_PIN	BIT2
#define LED_OFF			LED_RED_PORT &= ~(LED_RED_PIN + LED_GREEN_PIN);
#define LED_RED			LED_RED_PORT |= LED_RED_PIN; LED_GREEN_PORT &= ~LED_GREEN_PIN
#define LED_GREEN		LED_RED_PORT &= ~LED_RED_PIN; LED_GREEN_PORT |= LED_GREEN_PIN
#define LED_YELLOW		LED_RED_PORT |= (LED_RED_PIN + LED_GREEN_PIN)

#define TRF_ENABLE_SET	P4DIR |= BIT6		// P4.1 is switched in output direction
#define	TRF_ENABLE		P4OUT |= BIT6		// EN pin on the TRF7970A
#define TRF_DISABLE		P4OUT &= ~BIT6

// IRQ on 2.0
#define IRQ_PIN_SET		P2DIR &= ~IRQ_PIN;
#define IRQ_PIN         BIT0
#define IRQ_PORT		P2IN
#define IRQ_ON			P2IE |= IRQ_PIN
#define IRQ_OFF			P2IE &= ~IRQ_PIN
#define IRQ_EDGE_SET	P2IES &= ~IRQ_PIN		// Rising edge interrupt
#define IRQ_CLR			P2IFG = 0x00
#define IRQ_REQ_REG		P2IFG


#define SLAVE_SELECT_PORT_SET   P4DIR |= BIT0;
#define SLAVE_SELECT_HIGH       P4OUT |= BIT0;
#define SLAVE_SELECT_LOW        P4OUT &= ~BIT0;

#define SPI_PORT_FUNC1      P4SEL
#define SPI_PORT_FUNC2      P4SEL
#define SPI_DIR             P4DIR
#define SPI_OUT             P4OUT
#define SPI_CLK             BIT3
#define SPI_MISO            BIT2
#define SPI_MOSI            BIT1
#define SPI_CTL0			UCB1CTL0
#define SPI_CTL1			UCB1CTL1
#define SPI_BR0				UCB1BR0
#define SPI_BR1				UCB1BR1
#define SPI_RX_BUF			UCB1RXBUF
#define SPI_TX_BUF			UCB1TXBUF
#define SPI_RX_READY		UCB1IFG & UCRXIFG
#define SPI_TX_READY		UCB1IFG & UCTXIFG
#define SPI_TX_BUSY			UCB1STAT & UCBUSY

//-----Counter-timer constants-----------------------------------

#define COUNTER_VALUE       TA0CCR0                 //counter register
#define START_COUNTER       TA0CTL |=  MC1        	//start counter in up mode
#define STOP_COUNTER        TA0CTL &= ~(MC0 + MC1)  //stops the counter
#define RESET_COUNTER       TA0CTL |= TACLR	    	//Resets and stops counter.

#endif

#if defined (__MSP430G2553__)
#define TRF_ENABLE_SET	P2DIR |= BIT2		// P2.2 is switched in output direction
#define	TRF_ENABLE		P2OUT |= BIT2		// EN pin on the TRF7970A
#define TRF_DISABLE		P2OUT &= ~BIT2

// IRQ on P2.0
#define IRQ_PIN_SET		P2DIR &= ~IRQ_PIN;
#define IRQ_PIN			(BIT0 | BIT7)
#define IRQ_PORT		P2IN
#define IRQ_ON			P2IE |= IRQ_PIN
#define IRQ_OFF			P2IE &= ~IRQ_PIN
#define IRQ_EDGE_SET	P2IES &= ~IRQ_PIN		// Rising edge interrupt
#define IRQ_CLR			P2IFG = 0x00
#define IRQ_REQ_REG		P2IFG

#define LED_PORT_SET	P2DIR |= 0x38;
#define LED_ALL_OFF		P2OUT &= ~0x38;

#define LED_14443A_ON	P2OUT |= BIT4;
#define LED_14443A_OFF	P2OUT &= ~BIT4;
#define LED_14443B_ON	P2OUT |= BIT3;
#define LED_14443B_OFF	P2OUT &= ~BIT3;
#define LED_15693_ON	P2OUT |= BIT5;
#define LED_15693_OFF	P2OUT &= ~BIT5;

#define SLAVE_SELECT_PORT_SET	P2DIR |= BIT1;
#define SLAVE_SELECT_HIGH		P2OUT |= BIT1;
#define SLAVE_SELECT_LOW		P2OUT &= ~BIT1;

#define SPI_TX_READY		IFG2 & UCB0TXIFG
#define SPI_RX_READY		IFG2 & UCB0RXIFG
#define SPI_PORT_FUNC1      P1SEL
#define SPI_PORT_FUNC2      P1SEL2
#define SPI_DIR             P1DIR
#define SPI_OUT             P1OUT
#define SPI_CLK             BIT5
#define SPI_MISO            BIT6
#define SPI_MOSI            BIT7

#define UART_PORT_FUNC1		P1SEL
#define UART_PORT_FUNC2		P1SEL2
#define UART_TX				BIT2
#define UART_RX				BIT1
#define UART_TX_READY		IFG2 & UCA0TXIFG
#define UART_RX_READY		IFG2 & UCA0RXIFG

//-----Counter-timer constants-----------------------------------

#define COUNTER_VALUE	TACCR0					//counter register
#define START_COUNTER	TACTL |=  MC0			//start counter in up mode
#define STOP_COUNTER	TACTL &= ~(MC0 + MC1)	//stops the counter
#define RESET_COUNTER   TACTL |= TACLR	    	//Resets and stops counter.
#endif

#define COUNT_1ms		3		// Used for Timer Counter
#define DELAY_1ms		8000	// Used for McuDelayMillisecond


//===============================================================

#endif
