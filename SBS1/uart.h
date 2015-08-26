/*
	Author: Keenan Burnett

	***********************************************************************
	*	FILE NAME:		uart_lib.h
	*
	*	PURPOSE:	This is the header file for uart_lib.c
	*
	*	FILE REFERENCES:		
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
	*   http://www.avrfreaks.net/forum/usart-interrupt-atmega32m1
*/
#ifndef UART_H
#define UART_H

#define UART_BUFF_LEN	255
extern volatile uint8_t uart_buffer[UART_BUFF_LEN];
extern volatile uint8_t uart_index;
extern volatile uint8_t uart_overflow;

uint8_t uart_command[UART_BUFF_LEN];
uint8_t uart_buflen;				// This variable is used to store the current length of the uart buffer.
uint8_t uart_comlen;				// This variable is used to store the current length of the uart command.
uint8_t uart_listen;				// This flag is used to determine whether or not subsequent UART bytes correspond to a command.
uint8_t blinking;


/* Function Prototypes	*/
void uart_init(void);
uint8_t uart_transmit(uint8_t msg);
uint8_t uart_receive(void);
void uart_clear_buff(void);
void uart_clear_command(void);
void uart_read_command(void);

#endif