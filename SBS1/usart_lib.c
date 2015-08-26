/*
	Author: Keenan Burnett

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
*/
#include "usart_lib.h"

/************************************************************************/
/* UART INITIALIZE                                                      */
/*																		*/
/* This function initializes the registers which are required for UART	*/
/* communication.														*/
/************************************************************************/

//void uart_initialize(void)
//{
	//uint8_t* uart_ptr;
	//
	//uart_ptr = LINCR;
	//*uart_ptr = LINCRVAL;	// Set the configuration register.
//
	//uart_ptr = LINBRRL;
	//*uart_ptr = LINBRRLVAL;	// Set the BAUDR register.
	//
	//uart_clear_buff();
	//uart_clear_command();
	//uart_listen = 0;
	//
	//return;
//}

/************************************************************************/
/* UART SEND	                                                        */
/*																		*/
/* @purpose: This function sends an array of chars via UART.			*/
/* @param: *msg is an array of characters which is to be sent via UART	*/
/*		   length is the length of the *msg array.						*/
/* @retval: If 0 is returned, the operation succeeded, -1 indicates fail*/
/************************************************************************/

uint8_t uart_send(uint8_t* msg, uint8_t length)
{
	uint8_t* usart_ptr;
	uint8_t i, timeout = 800000, tx_flag;	// The timeout is 0.1s
		
	for(i = 0; i < length; i ++)
	{
		usart_ptr = LDAT;
		*(usart_ptr) = *(msg + i);
		
		usart_ptr = LINSIR;
		tx_flag = (*usart_ptr) * 0x02;
		
		while(!tx_flag)
		{
			if(!timeout--)
				return -1;					// The operation timed out.
		}
		timeout = 800000;
	}
	return 0;								// The operation succeeded.
}

/************************************************************************/
/* UART READ                                                            */
/*																		*/
/* @purpose: This function reads from UART and returns a single byte.	*/
/* @param: *status is a pointer used to indicate the status of the		*/
/*	operation, -1 indicates failure, 0 indicates success.				*/
/* @retval: returns the byte which was read (if success), ow 0.			*/
/************************************************************************/

uint8_t uart_read(uint8_t* status)
{
	uint8_t* uart_ptr;
	uint8_t rx_flag, ret_val;;
	
	uart_ptr = LINSIR;
	rx_flag = (*uart_ptr) & 0x01;
	
	if(!rx_flag)
	{
		*status = -1;
		return;
	}
	
	uart_ptr = LDAT;
	ret_val = *uart_ptr;
	*status = 0;
	
	return ret_val;
}

/************************************************************************/
/* UART CHECK                                                           */
/*																		*/
/* @purpose: This function shall be used to check to see whether a		*/
/* command has been received that needs to be acted upon.				*/
/* NOTE: In order for the bytes of a command to be stored, you must		*/
/* first send '1' and then finish the command with '2'					*/
/************************************************************************/
void uart_check(void)
{
	uint8_t* uart_ptr;
	uint8_t rx_flag, msg, status, i;
	
	uart_ptr = LINSIR;
	rx_flag = (*uart_ptr) & 0x01;
	
	if(rx_flag)
	{
		msg = uart_receive();
	}
	
	if(msg = 0x32)		// 0x32 == '2' in ASCII.
	{
		uart_listen = 0;
		for(i = 0; i < uart_buflen; i ++)
		{
			uart_command[i] = uart_buffer[i];
		}
		uart_comlen = uart_buflen;
		uart_clear_buff();
		uart_read_command();
	}
	
	if (uart_listen)
	{
		uart_buffer[uart_buflen] = msg;
		uart_buflen++;
	}
	
	if(msg = 0x31)		// 0x31 == '1' in ASCII.
	{
		uart_listen = 1;
		uart_clear_buff();
	}
	
	return;
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
		msg_array[2] = 0x0D;	// carriage return
		status = uart_send(response, (uint8_t)13);
		led_toggle(LED1);
		delay_ms(100);
		led_toggle(LED1);
		delay_ms(100);
	}
	
	// Add other possible commands below!
	
	uart_clear_command();
	
	return;
}

