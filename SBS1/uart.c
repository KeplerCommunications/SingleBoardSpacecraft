/*
	***********************************************************************
	*	FILE NAME:		uart_lib.c
	*
	*	PURPOSE:	This program contains functions related to UART communication
	*				in the ATMEGA32M1.
	*
	*	FILE REFERENCES:		uart_lib.h
	*
	*	EXTERNAL VARIABLES:		None.
	*
	*	EXTERNAL REFERENCES:	Same a File References.
	*
	*	ABORNOMAL TERMINATION CONDITIONS, ERROR AND WARNING MESSAGES: None.
	*
	*	ASSUMPTIONS, CONSTRAINTS, CONDITIONS:	None
	*
	*	NOTES:
	*
	*	REQUIREMENTS/ FUNCTIONAL SPECIFICATION REFERENCES:
	*	None so far.
	*
	*	DEVELOPMENT HISTORY:
	*	08/19/2015		Created.
	*
	* http://www.avrfreaks.net/forum/usart-interrupt-atmega32m1
*/
#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "led.h"

volatile uint8_t uart_buffer[UART_BUFF_LEN] = {0};
volatile uint8_t uart_index = 0;
volatile uint8_t uart_overflow = 0;

ISR (LIN_TC_vect)
{	
	uint8_t msg, i;
	
	msg = LINDAT;	// Incoming UART message.
	uart_overflow = uart_index >= UART_BUFF_LEN;

	if(msg == 0x32)		// 0x32 == '2' in ASCII.
	{
		uart_listen = 0;
		for(i = 0; i < uart_index; i ++)
		{
			uart_command[i] = uart_buffer[i];
		}
		uart_comlen = uart_buflen;
		uart_clear_buff();
		uart_read_command();
	}
		
	if (uart_listen)
	{
		uart_buffer[uart_index] = msg;
		if (uart_index == (UART_BUFF_LEN - 1))
		{
			uart_clear_buff();		// A message overflow has occurred, empty the buffer.
			return;
		}
		uart_index++;
	}
		
	if(msg == 0x31)		// 0x31 == '1' in ASCII.
	{
		uart_listen = 1;
		uart_clear_buff();
	}
	
	return;
}

/************************************************************************/
/* UART INITIALIZE                                                      */
/*																		*/
/* This function initializes the registers which are required for UART	*/
/* communication.														*/
/************************************************************************/

void uart_init(void)
{	
	// Initialize UART Registers
	LINCR = (1 << LSWRES);                    // Software reset
	LINBRRH = (((F_CPU/UART_BAUD)/16)-1)>>8;  // Baudrate top 8 bits
	LINBRRL = (((F_CPU/UART_BAUD)/16)-1);     // Baudrate lower 8 bits
	LINBTR = (1 << LDISR) | (1 << LBT4);      
	LINCR = (1<<LENA)|(1<<LCMD2)|(1<<LCMD1)|(1<<LCMD0); // Turn on UART for full duplex
	LINENIR = 0b00000001;                     // Set the ISR flags for just the receive
	LINSIR = 0b00000001;
	uart_clear_buff();
	uart_clear_command();
	uart_listen = 0;
	blinking = 0;
	
	sei();
}

 uint8_t uart_transmit (uint8_t msg) {
	 uint64_t timeout = F_CPU*30;
	 while ((LINSIR & (1 << LBUSY)) && (timeout--)); // Wait while the UART is busy.
	 LINDAT = msg;
	 return 0;
 }

 uint8_t uart_receive (void) {
	 uint64_t timeout = F_CPU*30;
	 while ((LINSIR & (1 << LBUSY)) && (timeout--)); // Wait while the UART is busy.
	 return LINDAT;
 }
 
 /************************************************************************/
 /* UART CLEAR BUFF                                                      */
 /*																		*/
 /* @purpose: This function is clears the buffer usart_buffer[]			*/
 /************************************************************************/
 void uart_clear_buff(void)
 {
	 uint8_t i;
	 
	 for (i = 0; i < UART_BUFF_LEN; i ++)
	 {
		 uart_buffer[i] = 0;
	 }
	 
	 uart_buflen = 0;
	 
	 return;
 }

 /************************************************************************/
 /* UART CLEAR COMMAND                                                   */
 /*																		*/
 /* @purpose: This function is clears the buffer usart_command[]			*/
 /************************************************************************/
 void uart_clear_command(void)
 {
	 uint8_t i;
	 
	 for (i = 0; i < UART_BUFF_LEN; i ++)
	 {
		 uart_command[i] = 0;
	 }
	 
	 uart_comlen = 0;
	 
	 return;
 }

 /************************************************************************/
 /* USART READ COMMAND                                                   */
 /*																		*/
 /* @purpose: This function checks whether the command which is stored in*/
 /* usart_command[] is one that needs to be acted upon. It then either	*/
 /* blinks an LED or sets a flag so that an action may be performed		*/
 /* elsewhere.															*/
 /************************************************************************/

 void uart_read_command(void)
 {
	 uint8_t i, blink = 1, status;
	 uint8_t check_array[25];
	 uint8_t msg_array[10];
	 uint8_t* response;
	 
	 // "BLINK"
	 check_array[0] = 0x42;
	 check_array[1] = 0x4C;
	 check_array[2] = 0x49;
	 check_array[3] = 0x4E;
	 check_array[4] = 0x4B;
	 
	 for(i = 0; i < 5; i ++)
	 {
		 if(check_array[i] != uart_command[i])
		 blink = 0;
	 }
	 
	 if(blink)	// "BLINK" was sent!
	 {
		 msg_array[0] = 0x0A;	// line feed
		 msg_array[1] = 0x0D;	// carriage return
		 msg_array[2] = 0x42;	// msg_array == "\n\rBLINKING!\n\r"
		 msg_array[3] = 0x4C;
		 msg_array[4] = 0x49;
		 msg_array[5] = 0x4E;
		 msg_array[6] = 0x4B;
		 msg_array[7] = 0x49;
		 msg_array[8] = 0x4E;
		 msg_array[9] = 0x47;
		 msg_array[10] = 0x21;
		 msg_array[11] = 0x0A;	// line feed
		 msg_array[12] = 0x0D;	// carriage return
		 
		 for (i = 0; i < 13 ; i ++)
		 {
			 status = uart_transmit(msg_array[i]);
		 }
		 
		 blinking = 1;
	 }
	 
	 // Add other possible commands below!
	 
	 uart_clear_command();
	 
	 return;
 }
