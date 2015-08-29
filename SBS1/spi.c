/*
	Author: 

	***********************************************************************
	*	FILE NAME:		spi.c
	*
	*	PURPOSE:	This program contains functions related to SPI communication
	*				in the ATMEGA32M1.
	*
	*	FILE REFERENCES:		spi_lib.h
	*
	*	EXTERNAL VARIABLES:		None.
*/

#include "spi.h"
#include "uart.h"
#include <util/delay.h>
#include <avr/io.h>

void spi_init(){
	// By default the MCUCR register has SPIPS set to 0 (use SPI instead of SPI_A)
	// we won't touch this because you shouldn't change the register with interrupts enabled
	DDRB |=   (1<<PB1)|(1<<PB7);
	DDRB &=   ~(1<<PB0);
	//SPCR  =   (1<<SPE)|(1<<MSTR)|(1<<SPR0)|(1<<SPR1);     // Enable SPI, and set as Master
	SPCR  =   (1<<SPE)|(1<<MSTR); 
}

/************************************************************************/
/*		SPI TRANSFER (as a master)                                      */
/*																		*/
/*		This function takes in a single byte as a parameter and then	*/
/*		proceeds to load this byte into the SPDR register in order to	*/
/*		initiate transmission. It will then loop until the transmission	*/
/*		is completed. If the transmission times out, it returns 0.		*/
/*		A successful transmission will return the byte which was		*/
/*		received on the MISO line during the transfer.					*/
/*																		*/
/************************************************************************/
uint8_t spi_transfer(uint8_t message){
	SPDR = message;
	// Wait till transmission is complete
	while(!(SPSR&(1<<SPIF))){
		if (SPSR&(1<<WCOL)){
			uart_sendmsg("Collision Detected!!\n");
			break;
		}
	}
	_delay_loop_1(30);
	return message = SPDR;
} 


/************************************************************************/
/*		SS_set_high                                                     */
/*																		*/
/*		This function is to be used to be used to simplify the command	*/
/*		that is required when the SS needs to be set high during an		*/
/*		SPI message.													*/
/*																		*/
/************************************************************************/
void SS_set_high(void){
	//PORTD |= (1 << 3);
	_delay_us(1);
}

/************************************************************************/
/*		SS_set_low	                                                    */
/*																		*/
/*		This function is to be used to be used to simplify the command	*/
/*		that is required when the SS needs to be set low during an		*/
/*		SPI message.													*/
/*																		*/
/************************************************************************/
void SS_set_low(void){
	//PORTD &= (0xF7);
	_delay_us(1);
}


