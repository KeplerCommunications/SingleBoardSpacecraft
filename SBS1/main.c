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
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "led.h"
#include "uart.h"

/* Function Prototypes for functions in this file */
//static void io_init(void);
static void sys_init(void);
/**************************************************/

volatile uint8_t CTC_flag;	// Variable used in timer.c
uint8_t status;
uint8_t* msg;

int main(void)
{		
	// Initialize I/O, Timer, ADC, CAN, and SPI
	sys_init();
	led_init();
	uart_init();
	
	/*		Begin Main Program Loop					*/	
    while(1){	
		led_toggle(1);
		_delay_ms(100);		
		led_toggle(2);
		_delay_ms(200);
		if (uart_index){
			uint8_t i;
			for (i = 0; i < uart_index; i++){
				uart_transmit(uart_buffer[i]+1);
			}
			uart_index -= i;
		}
	}
}

void sys_init(void) 
{
	// Make sure sys clock is at least 8MHz
	CLKPR = 0x80;  
	CLKPR = 0x00;
	
	//io_init();	
	
	//timer_init();
	//spi_initialize_master();
	//uart_initialize();
	
	// Enable global interrupts for Timer execution
	//sei();
	
	//transceiver_initialize();
	
	//SS1_set_high();		// SPI Temp Sensor.
	
	//LED_toggle(LED7);
	//LED_set(LED1);
	//LED_set(LED2);
	//LED_set(LED3);
	//PORTD &= ~(1<<1);
}
