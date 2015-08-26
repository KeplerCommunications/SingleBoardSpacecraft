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
*/

#include "config.h"
#include "LED.h"
#include "uart.h"

/*	REGISTER DEFINITIONS	*/
//#define LINCR		0xC8	// LIN Control Register
//#define LINSIR		0xC9	// LIN Status & Interrupt Register
//#define LINBTR		0xCC	// LIN Enable Interrupt Register (default = 32)
//#define LINBRRH		0xCE	// LIN Bit Timing Register High
//#define LINBRRL		0xCD	// " " Low
#define LDAT		0xD2	// LIN Data Register
//
///* REGISTER CONFIGURATION VALUES	*/
//#define LINCRVAL	0x08
///* [5,4] = LCONF[1,0] = 00 => UART: 8-BIT, No parity, 1 stop-bit
   //[3] = LENA = 1 => UART Enabled
   //[2-0] = LCMD = 000 => Full Duplex */
//#define LINBRRLVAL	103		// 103 corresponds to a BAUDR of 2400
//
///* Global Variables	*/
//#define UART_BUFF_LEN	25
//uint8_t uart_buffer[UART_BUFF_LEN];	// This is to be used to store the UART message which is received.
uint8_t uart_command[UART_BUFF_LEN];
uint8_t uart_buflen;				// This variable is used to store the current length of the uart buffer.
uint8_t uart_comlen;				// This variable is used to store the current length of the uart command.
uint8_t uart_listen;				// This flag is used to determine whether or not subsequent UART bytes correspond to a command.

/* Function Prototypes	*/
//void uart_initialize(void);
uint8_t uart_send(uint8_t* msg, uint8_t length);
uint8_t uart_read(uint8_t* status);
void uart_check(void);
void uart_clear_buff(void);
void uart_clear_command(void);
void uart_read_command(void);
