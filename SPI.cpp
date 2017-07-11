/*
 * SPI.cpp
 *
 * Created: 07.05.2017 14:43:16
 *  Author: JackFrost
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

void SPIC_Init(void)
{
	
	
	// PA1 output (SS CAN)
	PORTA.DIRSET = PIN1_bm;
	// PC2 output (SS FLASH)
	PORTC.DIRSET = PIN2_bm;
	// PF4 input (SS CAN) Test 
	PORTC.DIRSET = PIN4_bm;
	// PF5 output (MOSI SPIC)
	PORTC.DIRSET = PIN5_bm;
	// PF6 input (MISO SPIC)
	PORTC.DIRCLR = PIN6_bm;
	// PF7 output (SCK SPIC)
	PORTC.DIRSET = PIN7_bm;
	
	//SS disabled (high)
	PORTA.OUTSET = PIN1_bm;
	PORTC.OUTSET = PIN2_bm;
	
	//Prescaler at 16 and CLK2X off  --> CLK = 1MHz
	//Mode 0 (CPOL=0 CPHA=0)
	//Master mode
	SPIC.CTRL = SPI_MODE_0_gc | SPI_PRESCALER_DIV4_gc | SPI_ENABLE_bm | SPI_MASTER_bm ;
	_delay_ms(1);
	
}

unsigned char SPIC_Read_Write(unsigned char data)
{
	SPIC.DATA = data; //send
	while(!(SPIC.STATUS & (1<<SPI_IF_bp))); //wait completion
	return SPIC.DATA;
	
}

void SPIC_Write(unsigned char data)
{
	SPIC.DATA = data; //send
	while(!(SPIC.STATUS & (1<<SPI_IF_bp))); //wait completion
	uint8_t dummy = SPIC.DATA;

	
}

void SPIC_Write_array(uint8_t *data, uint8_t length)
{
	for(uint8_t i = 0; i<length;i++)
	{
		SPIC.DATA = data[i]; //send
		while(!(SPIC.STATUS & (1<<SPI_IF_bp))); //wait completion
		uint8_t dummy = SPIC.DATA;
	}
}

void SPIC_Read_array(uint8_t *data, uint8_t length)
{
	for(uint8_t i = 0; i<length;i++)
	{
		SPIC.DATA = 0xFF; //send
		while(!(SPIC.STATUS & (1<<SPI_IF_bp))); //wait completion
		data[i] = SPIC.DATA;
	}
}