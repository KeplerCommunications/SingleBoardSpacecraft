/*
    Author: Keenan Burnett

	***********************************************************************
	*	FILE NAME:		main.c
	*
	*	PURPOSE:		
	*	This is the main program which shall be run on the ATMEGA32M1s to be used on subsystem
	*	microcontrollers.
	*
	*	FILE REFERENCES:	io.h, interrupt, LED.h, Timer.h, spi_lib.h, trans_lib.h
	*
	*	EXTERNAL VARIABLES:	
	*
	*	EXTERNAL REFERENCES:	Same a File References.
	*
	*	ABORNOMAL TERMINATION CONDITIONS, ERROR AND WARNING MESSAGES: None yet.
	*
	*	ASSUMPTIONS, CONSTRAINTS, CONDITIONS:	None
	*
	*	NOTES:	
	*
	*	REQUIREMENTS/ FUNCTIONAL SPECIFICATION REFERENCES:
	*	None so far.
	*
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "LED.h"
#include "Timer.h"
#include "spi_lib.h"
#include "trans_lib.h"
#include "uart_lib.h"

/* Function Prototypes for functions in this file */
static void io_init(void);
static void sys_init(void);
/**************************************************/

volatile uint8_t CTC_flag;	// Variable used in timer.c
uint8_t status;
uint8_t* msg;

int main(void)
{		
	// Initialize I/O, Timer, ADC, CAN, and SPI
	sys_init();
	/*		Begin Main Program Loop					*/	
    while(1)
    {		
		/*		TRANSCEIVER COMMUNICATION	*/
		trans_check();
		
		if(msg_received)
		{
			// Do stuff with the message in trans_msg[].
			
			// Send the message to laptop via UART
		}
		uart_check();
		
	}
}

void sys_init(void) 
{
	// Make sure sys clock is at least 8MHz
	CLKPR = 0x80;  
	CLKPR = 0x00;
	
	io_init();	
	
	timer_init();
	spi_initialize_master();
	uart_initialize();
	
	// Enable global interrupts for Timer execution
	sei();
	
	transceiver_initialize();
	
	SS1_set_high();		// SPI Temp Sensor.
	
	LED_toggle(LED7);
}

void io_init(void) 
{	
	// Init PORTB[7:0] // LED port
	DDRB = 0xFE;
	
	// Init PORTC[7:0] // PORTC[3:2] => RXCAN:TXCAN
	DDRC = 0x11;		// PC0 = SS for SPI with transceiver.
	PORTC = 0x00;
	
	// Init PORTD[7:0]
	DDRD = 0x09;		// PD3 = TXLIN, PD4 = RXLIN
	PORTD = 0x01;		// PD3 should only go low during an SPI message.
	
	// Init PORTE[2:0]
	DDRE = 0x00;
	PORTE = 0x00;
}

