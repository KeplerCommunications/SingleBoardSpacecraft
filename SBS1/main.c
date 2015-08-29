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
#include <string.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "transceiver.h"
#include "led.h"
#include "spi.h"
#include "uart.h"
#include "wdt.h"

/* Function Prototypes for functions in this file */
static void usr_serial_cmd();
/**************************************************/

volatile uint8_t CTC_flag;	// Variable used in timer.c
uint8_t status;
uint8_t* msg;

int main(void){
	CLKPR |= 1<<CLKPCE; // Enable clock pre-scaler change
	CLKPR  = 0;         // Set clock pre-scaler to 1 (fast as possible)
	cli();
	// Initialize interrupt based services first
	wdt_init();
	spi_init();
	uart_init();
	sei();
	long int lastStatus = 0;
	
	_delay_ms(100); // wait for uart to initialize

	uart_sendmsg("Starting initialize... \n");
	// Initialize remainder of services
	led_init();
	led_toggle(1);
	uart_sendmsg("Completed LED initialize... \n");
	transceiver_init(1);
	uart_sendmsg("Completed TRS initialize... \n");
	/*		Begin Main Program Loop					*/	
    while(1){
		usr_serial_cmd();
		transceiver_run();
		if ((millis()-lastStatus) >= 1000){
			led_toggle(2);
			led_toggle(1);
			transceiver_printStatus();
			lastStatus = millis();
			#if DEBUG_MODE
			uart_debug();
			#endif
		}
	}
}

void usr_serial_cmd(){
  //inputs command from serial monitor
  if(uart_index){
	int count = 0;
	uint8_t isCommand = 0;
	uint8_t msgLen;
	for(msgLen = 0; msgLen <= uart_index; msgLen++){
		if (uart_buffer[msgLen] == '!'){
			isCommand = 1;
			break;
		}
	}
	if (isCommand == 0)
		return;
	uart_buffer[msgLen] = 0;
	char cmd[128] = {0};
	char cD1[128] = {0} ;
	char cD2[1] = {0} ;
	// Got the reset command
	if(strcmp("RESET", (char*)uart_buffer) == 0){
		transceiver_init(0);
	}
	else if ((count = sscanf((char*)uart_buffer, "%[^:]:%*c%[^:]%*c%*c%c", cmd, cD1, cD2)) >= 2){
		char* message = 0;
		uint8_t destination = 0;
		// If we just have a send we will broadcast
		if (strcmp(cmd,"SEND")== 0 && count == 2){
			message = cD1;
			destination = 0xFF; // Broadcast
		}
		// Otherwise get the address and send to that
		else if (strcmp(cmd, "SEND") ==  0 && cD1[strlen(cD1)-2] == 'T' && cD1[strlen(cD1)-1] == 'O'){
			cD1[strlen(cD1)-3] = '\0';
			message = cD1;
			destination = cD2[0];
		}
		if (strlen(message) > 0 && strlen(message) < 0x7E){
			uart_sendmsg("\n*** Sending \""); 
			uart_sendmsg(message); // Not using printf since message might be too long
			uart_transmit('"');
			if (count > 2){
				uart_printf(" to address: %d", destination);
			}
			uart_sendmsg(" ***\n");
			transceiver_send((uint8_t*)message, destination, strlen(message));
		}
		else {
			uart_sendmsg("Error: processing send command\n");
		}
	}
	else {
		uart_printf("Error: Invalid command\n");
	}
	// Reset the UART Buffer. WARNING: if we get a msg as this happens it will be corrupt
	// memcpy((void*)uart_buffer, (void*)uart_buffer+msgLen+1, UART_BUFF_LEN-(msgLen+1));
	for (count = 0; count < UART_BUFF_LEN - (msgLen + 1); count++){
		uart_buffer[count] = uart_buffer[count+msgLen+1];
		uart_buffer[count+msgLen + 1] = 0;
	}
	uart_index -= msgLen + 1;
  }	
}