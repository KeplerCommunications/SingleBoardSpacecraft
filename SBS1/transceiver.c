/*
	Authors: 

	***********************************************************************
	*	FILE NAME:		transceiver.c
	*
	*	PURPOSE:	This program contains functions related to the communications transceiver.
	*
	*	FILE REFERENCES:		transceiver.h
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
*/
#include "config.h"
#include <util/delay.h>
#include <stdio.h>
#include "spi.h"
#include "uart.h"
#include "wdt.h"
#include "led.h"
#include "transceiver.h"

unsigned long previousTime = 0;
unsigned long currentTime = 0;
long int lastTransmit = 0;
long int lastToggle = 0;
uint8_t tx_mode = 0;
uint8_t rx_mode = 1;
//char message[128];
//uint8_t address;




/************************************************************************/
/*		REG_SETTING                                                     */
/*																		*/
/*		This function is used to setup register settings on	            */
/*		 CC1120 transceiver												*/
/*																		*/
/************************************************************************/


void transceiver_init(uint8_t calibrate){
	if (calibrate){
		DDRD |= (1<<PD1);
		DDRB |= (1<<PB5);
		DDRC |= (1<<PC4);
		PORTD |= (1<<PD1);
		PORTB |= (1<<PB5);
		PORTC |= (1<<PC4);
		_delay_ms(1000);
		PORTC &= ~(1<<PC4);
		_delay_ms(1000);
		PORTC |= (1<<PC4);
		PORTD &= ~(1<<PD1);
	}
	// SPI is already in MSB first, which is correct for the CC1120.
    
    cmd_str(SRES,DEBUG_MODE);  //SRES                  reset chip
    cmd_str(SFRX,DEBUG_MODE);  //SFRX                  flush RX FIFO
    cmd_str(SFTX,DEBUG_MODE);  //SFTX                  flush TX FIFO
  	reg_setting();			   //CC1120 register setting

	if (calibrate){
		//strobe commands to start RX
		cmd_str(SCAL,DEBUG_MODE);    // Calibrate frequency synthesizer
		_delay_ms(250);
		cmd_str(SAFC,DEBUG_MODE);    // Automatic frequency control
		_delay_ms(250);
	}
	rx_mode = 1;
	tx_mode = 0;
	prepareAck();
	if(!TX_ONLY_MODE)
		cmd_str(SRX,DEBUG_MODE);         // Put in RX mode 
	
	if(TX_ONLY_MODE){
		// Code to TX 'a' continuously
		// fill the TX FIFO
		for (int i = 0; i<0x100;i++){
			dir_FIFO_write(i,0x41);
		}
		// set up TX FIFO pointers
		reg_write2F(TXFIRST,0);
		reg_write2F(TXLAST,0x7F);
		cmd_str(STX,DEBUG_MODE); // Put in TX Mode
	}
	
}

void transceiver_run(){
	// Get the current state
	uint8_t state;
	uint8_t CHIP_RDYn;
	get_status(&CHIP_RDYn, &state);
	
	if(!TX_ONLY_MODE){	
		// Waited too long, resend
		if (millis()-lastTransmit>= ACK_TIMEOUT){
			//led_toggle(3);
			lastTransmit = millis();
		}
	
		// Waiting for acknowledge didn't get it yet
		if (tx_mode && state == STATERX){
			uint8_t rxFirst = reg_read2F(RXFIRST);
			if (rxFirst){
				lastTransmit = millis();
				reg_write2F(RXFIRST,0);
				reg_write2F(RXLAST,0);
			}
			// Waited too long, resend
			if (millis()-lastTransmit>= ACK_TIMEOUT){
				uart_sendmsg("***ACK Wait Timeout, Retransmitting Message***\n");
				reg_write2F(TXFIRST, 0x00);         //set TX FIRST to 0
				cmd_str(STX, DEBUG_MODE);                    //put in TX mode
				lastTransmit = millis();
			}
		 }
		  // Still sending the data that we have
		 else if (tx_mode && state == STATETX){
			  uint8_t rxFirst = reg_read2F(RXFIRST);
			  if (rxFirst){
				  reg_write2F(RXFIRST,0);
				  reg_write2F(RXLAST,0);
				  lastTransmit = millis();
			  }
		  }
		  // Waiting for data to come
		  else if (rx_mode){
			  // Get the data from the FIFO
			  uint8_t rxFirst = reg_read2F(RXFIRST);
			  uint8_t rxLast = reg_read2F(RXLAST);
			  led_toggle(1);
			  // Got some data
			  if (rxFirst <= rxLast && rxLast){
				  //led_toggle(3);
				  uint8_t fifo[128] = {0};
				  uint8_t j = 0;
				  for (uint8_t i = rxFirst; i < rxLast; i++){
					  fifo[j++] = dir_FIFO_read(i);
				  }
				  // We have a packet
				  if (fifo[0] <= (rxLast - rxFirst - 1)){
					  uart_printf("Got packet for address: %d msg: \"", fifo[1]);
					  for (j = 1; j < fifo[0]; j++){
						  uart_printf("%c", fifo[j+1]);
					  }
					  uart_printf("\"\n");
					  reg_write2F(RXFIRST, fifo[0]);
					  rxFirst += fifo[0];
				  }
				  // The packet doesn't seem to be done
				  else if (fifo[0] >= (rxLast - rxFirst - 1)){
				  }
			  }
			  if (rxFirst - rxLast == 0 && rxFirst){
				  reg_write2F(RXFIRST, 0x00);
				  reg_write2F(RXLAST, 0x00);
			  }
			  reg_write2F(TXFIRST, 0); // So we can send another ACK
		  }
		  // Something went wrong
		  else {
		  }
	  }
	  // TX only mode
	  else{
		  if(reg_read2F(TXFIRST)>0x7E){
			  uart_sendmsg("*** FIFO RESET ***\n");
			  cmd_str(SIDLE,DEBUG_MODE);
			  cmd_str(SFTX,DEBUG_MODE);
			  reg_write2F(TXFIRST,0);
			  reg_write2F(TXLAST,0x7F);
			  cmd_str(STX,DEBUG_MODE);
		  }
	  }
}

// setup register settings
void reg_setting(){
	//NOTE: This is based on SmartRF settings
	// 1. Open the device in smartRF
	// 2. 'Register Reset'
	// 3. High performance mode
	// 4. The regs should match below

	//high performance settings
	reg_write2F(0x12, 0x00);          //FS_DIG1: 0x00         Frequency Synthesizer Digital Reg. 1
	reg_write2F(0x13, 0x5F);          //FS_DIG0: 0x5F         Frequency Synthesizer Digital Reg. 0
	reg_write2F(0x16, 0x40);          //FS_CAL1: 0x40         Frequency Synthesizer Calibration Reg. 1
	reg_write2F(0x17, 0x0E);          //FS_CAL0: 0x0E         Frequency Synthesizer Calibration Reg. 0
	reg_write2F(0x19, 0x03);          //FS_DIVTWO: 0x03       Frequency Synthesizer Divide by 2
	reg_write2F(0x1B, 0x33);          //FS_DSM0: 0x33         FS Digital Synthesizer Module Configuration Reg. 0
	reg_write2F(0x1D, 0x17);          //FS_DVCO: 0x17         Frequency Synthesizer Divider Chain Configuration ..
	reg_write2F(0x1F, 0x50);          //FS_PFD: 0x50          Frequency Synthesizer Phase Frequency Detector Con..
	reg_write2F(0x20, 0x6E);          //FS_PRE: 0x6E          Frequency Synthesizer Prescaler Configuration
	reg_write2F(0x21, 0x14);          //FS_REG_DIV_CML: 0x14  Frequency Synthesizer Divider Regulator Configurat..
	reg_write2F(0x22, 0xAC);          //FS_SPARE: 0xAC        Set up Frequency Synthesizer Spare
	reg_write2F(0x27, 0xB4);          //FS_VCO0: 0xB4         FS Voltage Controlled Oscillator Configuration Reg..
	reg_write2F(0x32, 0x0E);          //XOSC5: 0x0E           Crystal Oscillator Configuration Reg. 5
	reg_write2F(0x36, 0x03);          //XOSC1: 0x03           Crystal Oscillator Configuration Reg. 0
	
	//modulation and freq deviation settings
	reg_write(0x0A, 0b01001000);       //DEVIATION_M: 0x48      set DEV_M to 72 which sets freq deviation to 20.019531kHz (with DEV_M=5)
	reg_write(0x0B, 0b00000101);       //MODCFG_DEV_E: 0x05     set up modulation mode and DEV_E to 5 (see DEV_M register)
	reg_write(0x21, 0b00000100);       //FS_CFG: B00010100      set up LO divider to 8 (410.0 - 480.0 MHz band), out of lock detector disabled
	
	//set preamble
	reg_write(0x0D, 0x00);            //PREAMBLE_CFG1: 0x00    No preamble
	reg_write_bit(0x0E, 5, 0);        //PQT_EN: 0x00           Preamble detection disabled
	
	//TOC_LIMIT
	reg_write_bit2F(0x02, 7, 0);        //TOC_LIMIT: 0x00      Using the low tolerance setting (TOC_LIMIT = 0) greatly reduces system settling times and system power consumption as no preamble bits are needed for bit synchronization or frequency offset compensation (4 bits preamble needed for AGC settling).
	reg_write_bit2F(0x02, 6, 0);        //TOC_LIMIT: 0x00      Using the low tolerance setting (TOC_LIMIT = 0) greatly reduces system settling times and system power consumption as no preamble bits are needed for bit synchronization or frequency offset compensation (4 bits preamble needed for AGC settling).
	
	//set SYNC word
	reg_write_bit(0x08, 6, 0);        //PQT_GATING_EN: 0       PQT gating disabled (preamble not required)
	reg_write(0x09, 0x17);            //SYNC_CFG0: B00010111   32 bit SYNC word. Bit error qualifier disabled. No check on bit errors
	reg_write(0x04, 0x93);            //SYNC3: 0x93            Set SYNC word bits 31:24
	reg_write(0x05, 0x0B);            //SYNC2: 0x0B            Set SYNC word bits 23:16
	reg_write(0x06, 0x51);            //SYNC1: 0x51            Set SYNC word bits 15:8
	reg_write(0x07, 0xDE);            //SYNC0: 0xDE            Set SYNC word bits 7:0
		
	cmd_str(SNOP, 0);
	//set packets
	reg_write_bit(0x12, 6, 1);         //FIFO_EN: 0             FIFO enable set to true
	reg_write_bit(0x13, 6, 0);         //TRANSPARENT_MODE_EN: 0 Disable transparent mode
	reg_write(0x26, 0b00000000);       //PKT_CFG2: 0x00         set FIFO mode
	reg_write(0x27, 0b00000000);       //PKT_CFG1: 0x30         set address check and 0xFF broadcast
	reg_write(0x28, 0b00100000);       //PKT_CFG0: 0x30         set variable packet length
	reg_write(0x2E, 0x7F);             //PKT_LEN: 0xFF          set packet max packet length to 0x7F
	reg_write(0x1F, DEVICE_ADDRESS);   //DEV_ADDR register is set to DEVICE_ADDRESS
	reg_write(0x29, 0b00101110);       //RFEND_CFG1: 0x2E       go to TX after a good packet
	//reg_write(0x29, 0b00111110);     //RFEND_CFG1: 0x3E       go to RX after a good packet
	reg_write(0x2A, 0b00110000);       //RFEND_CFG0: 0x30       go to RX after transmitting a packet
	//reg_write(0x2A, 0b00100000);     //RFEND_CFG0: 0x20       go to TX after transmitting a packet
	
	//set power level
	reg_write(0x2B, 0b01111111);       //PA_CFG2: 0x7F          set POWER_RAMP to 64 (output power to 14.5dBm, equation 21)
	
	//frequency offset setting
	reg_write2F(0x0A, 0);             //FREQOFF1: 0x00         set frequency offset to 0
	reg_write2F(0x0B, 0);             //FREQOFF0: 0x00
	
	//Frequency setting
	reg_write2F(0x0C, 0x6C);          //FREQ2: 0x6C            set frequency to 434MHz (sets Vco, see equation from FREQ2 section of user guide)
	reg_write2F(0x0D, 0x80);          //FREQ1: 0x80
	reg_write2F(0x0E, 0x00);          //FREQ0: 0x00

}

/************************************************************************/
/*		REG_WRITE2F                                                     */
/*																		*/
/*		This function is used to read a single byte of data from an		*/
/*		address on the CC1120 transceiver (extended address)		    */
/*																		*/
/************************************************************************/

uint8_t reg_read(uint8_t addr)
{
	uint8_t addr_new, msg;
	addr_new = addr + 0b10000000;

	SS_set_low();
	msg = spi_transfer(addr_new);		// Send the desired address
	_delay_us(1);
	msg = spi_transfer(0x00);

	SS_set_high();
	
	_delay_ms(1);
	
	return msg;
}

/************************************************************************/
/*		REG_WRITE                                                       */
/*																		*/
/*		This function is used to write a single byte of data to an		*/
/*		address on the CC1120 transceiver (non-extended address)		*/
/*																		*/
/************************************************************************/

void reg_write(uint8_t addr, uint8_t data)		// Doesn't need to return anything.
{
	
	SS_set_low();
	spi_transfer(addr);		// Send the desired address
	_delay_us(1);
	spi_transfer(data);		// Send the desired data
	SS_set_high();
	_delay_ms(1);

	return;
}

/************************************************************************/
/*		REG_WRITE2F                                                     */
/*																		*/
/*		This function is used to read a single byte of data from an		*/
/*		address on the CC1120 transceiver (extended address)		    */
/*																		*/
/************************************************************************/

uint8_t reg_read2F(uint8_t addr)
{
	uint8_t msg;
	msg = 0b10101111;
	
	SS_set_low();
	msg = spi_transfer(msg);
	_delay_us(1);
	msg = spi_transfer(addr);		// Send the desired address
	_delay_us(1);
	msg = spi_transfer(0x00);
	SS_set_high();
	_delay_ms(1);
	return msg;
}

/************************************************************************/
/*		REG_WRITE2F                                                     */
/*																		*/
/*		This function is used to write a single byte of data to an		*/
/*		address on the CC1120 transceiver (extended address)		    */
/*																		*/
/************************************************************************/

void reg_write2F(uint8_t addr, uint8_t data)		// Doesn't need to return anything.
{
	cmd_str(SNOP,0);
	uint8_t msg;
	msg = 0b00101111;
	
	SS_set_low();
	spi_transfer(msg);
	_delay_us(1);
	spi_transfer(addr);		// Send the desired address
	_delay_us(1);
	spi_transfer(data);		// Send the desired data
	SS_set_high();
	_delay_ms(1);

	return;
}

/************************************************************************/
/*		GET_STATUS                                                      */
/*																		*/
/*		This function returns the 3 bits which correspond to the status	*/
/*		byte on the CC1120.												*/
/*		Note: 000 = IDLE, 001 = RX, 110 = RX buffer overflow.			*/
/*																		*/
/************************************************************************/

void get_status(uint8_t *CHIP_RDYn, uint8_t *state)
{
	uint8_t msg;
	msg = cmd_str(SNOP,0);
	*CHIP_RDYn = (msg>>7)&1;
	*state = (msg>>4)&7;
	//*state = (msg & ~0x80)>>4;
	return;
}

/************************************************************************/
/*	CMD_STR                                                             */
/*																		*/
/*	This function is used to send a single command over spi to the		*/
/*	CC1120.																*/
/*																		*/
/************************************************************************/

uint8_t cmd_str(uint8_t addr, uint8_t print)
{
	uint8_t msg;
	
	if(print){
		uart_sendmsg("Command strobe(");
		switch(addr){
			case SRES: uart_sendmsg("0x30): SRES"); break;
			case SFSTXON: uart_sendmsg("0x31): SFSTXON"); break;
			case SXOFF: uart_sendmsg("0x32): SXOFF"); break;
			case SCAL: uart_sendmsg("0x33): SCAL"); break;
			case SRX: uart_sendmsg("0x34): SRX"); break;
			case STX: uart_sendmsg("0x35): STX"); break;
			case SIDLE: uart_sendmsg("0x36): SIDLE"); break;
			case SAFC: uart_sendmsg("0x37): SAFC"); break;
			case SWOR: uart_sendmsg("0x38): SWOR"); break;
			case SPWD: uart_sendmsg("0x39): SPWD"); break;
			case SFRX: uart_sendmsg("0x3A): SFRX"); break;
			case SFTX: uart_sendmsg("0x3B): SFTX"); break;
			case SWORRST: uart_sendmsg("0x3C): SWORRST"); break;
			case SNOP: uart_sendmsg("0x3D): SNOP"); break;
		}
		uart_transmit('\n');
	}
	
	msg = spi_transfer(addr);
	
	_delay_us(1);
	return msg;
}

/************************************************************************/
/*	DIR_FIFO_READ                                                       */
/*																		*/
/*	This function takes in an address which corresponds to somewhere	*/
/*	on the CC1120's FIFO and return the byte which was located there.	*/
/*																		*/
/************************************************************************/

uint8_t dir_FIFO_read(uint8_t addr)
{
	cmd_str(SNOP,0);
	uint8_t msg;
	msg = 0b10111110;
	
	SS_set_low();
	msg = spi_transfer(msg);
	_delay_us(1);
	msg = spi_transfer(addr);		// Send the desired address
	_delay_us(1);
	msg = spi_transfer(0x00);
	SS_set_high();
	_delay_ms(1);
	return msg;
}

/************************************************************************/
/*	DIR_FIFO_READ                                                       */
/*																		*/
/*	This function takes in an address which corresponds to somewhere	*/
/*	on the CC1120's FIFO and a byte of data which is written to the FIFO*/
/*																		*/
/************************************************************************/

void dir_FIFO_write(uint8_t addr, uint8_t data)
{
	cmd_str(SNOP,0);
	uint8_t msg;
	msg = 0b00111110;
	
	SS_set_low();
	spi_transfer(msg);
	_delay_us(1);
	msg = spi_transfer(addr);		// Send the desired address
	_delay_us(1);
	msg = spi_transfer(data);		// Send the desired data
	SS_set_high();
	_delay_ms(1);
	
	return;
}

/************************************************************************/
/*		set_CSn                                                         */
/*																		*/
/*		This function sets the chip select pin (SS) to either LOW or	*/
/*		HIGH depending on param: state.									*/
/*																		*/
/************************************************************************/

void set_CSn(uint8_t state)
{
	if(state)
	SS_set_high();
	else
	SS_set_low();
}

/************************************************************************/
/*		REG_WRITE_BIT                                                   */
/*																		*/
/*		This function is used to write a single bit of data to an		*/
/*		address on the CC1120 transceiver (non-extended address)		*/
/*																		*/
/************************************************************************/

void reg_write_bit(uint8_t reg, uint8_t n, uint8_t data)
{
	uint8_t msg, temp;
	msg = reg_read(reg);
	if(!data)
	{
		temp = ~(1 << n);
		msg = temp & msg;
	}
	else
	{
		temp = 1 << n;
		msg = temp | msg;
	}
	reg_write(reg, msg);
	return;
}

/************************************************************************/
/*		REG_WRITE_BIT                                                   */
/*																		*/
/*		This function is used to write a single bit of data to an		*/
/*		address on the CC1120 transceiver (extended address)			*/
/*																		*/
/************************************************************************/

void reg_write_bit2F(uint8_t reg, uint8_t n, uint8_t data)
{
	uint8_t msg, temp;
	msg = reg_read2F(reg);
	if(!data)
	{
		temp = ~(1 << n);
		msg = temp & msg;
	}
	else
	{
		temp = 1 << n;
		msg = temp | msg;
	}
	reg_write2F(reg, msg);
	return;
}

void prepareAck(){
	char* ackMessage = "ACK";
	uint8_t ackAddress = 0xFF;
	
	// Reset FIFO registers
	reg_write2F(TXFIRST, 0x00);
	// Put the ACK Packet in the FIFO
	dir_FIFO_write(0,(uint8_t)4);
	dir_FIFO_write(1,ackAddress);
	
	for(uint8_t i=0; i<3; i++) 
		dir_FIFO_write(i+2, ackMessage[i]);
	
	reg_write2F(TXLAST, (uint8_t) 3 + 2);
	reg_write2F(RXFIRST, 0x00);
	reg_write2F(RXLAST, 0x00);
}

void transceiver_send(uint8_t* message, uint8_t address, uint8_t length){
	// The first byte is the length of the packet (message + 1 for the address)
	dir_FIFO_write(0,length+1);
	// The second byte is the address
	dir_FIFO_write(1,address);
	// The rest is the actual data
	for(int i=0x00; i<length; i++)
		dir_FIFO_write(i+2, message[i]);
	//set up TX FIFO pointers
	reg_write2F(TXFIRST, 0x00);            //set TX FIRST to 0
	reg_write2F(TXLAST, length+2);         //set TX LAST (maximum OF 0X7F)
	reg_write2F(RXFIRST, 0x00);            //set RXFIRST
	reg_write2F(RXLAST, 0x00);             //set RXLAST
	//strobe commands to start TX
	cmd_str(STX,DEBUG_MODE);                       //put in TX mode
	tx_mode = 1;
	lastTransmit = millis();           
}

//parses chip status byte
void transceiver_printStatus(){
	uint8_t CHIP_RDYn, state;
	char msg[10] = {0};
	get_status(&CHIP_RDYn, &state);
	
	uart_sendmsg("Chip status: ");
	if(CHIP_RDYn)
		uart_sendmsg("NOT READY (1), ");
	else
		uart_sendmsg("READY (0), ");
	
	//parses state
	switch (state){
		case 0b000: uart_sendmsg("IDLE (000)");      break;
		case 0b001: uart_sendmsg("RX (001)");        break;
		case 0b010: uart_sendmsg("TX (010)");        break;
		case 0b011: uart_sendmsg("FSTXON (011)");    break;
		case 0b100: uart_sendmsg("CALIBRATE (100)"); break;
		case 0b101: uart_sendmsg("SETTLING (101)");  break;
		case 0b110: uart_sendmsg("RX FIFO ERROR (110)");  break;
		case 0b111: uart_sendmsg("TX FIFO ERROR (111)");  break;
	}
	sprintf(msg,"at %lds\n",millis()/1000);
	uart_sendmsg(msg);
	return;
}