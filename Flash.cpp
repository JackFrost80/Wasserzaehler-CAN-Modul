/*
 * Flash.cpp
 *
 * Created: 18.06.2017 16:26:33
 *  Author: JackFrost
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include "Flash.h"
#include "SPI.h"

void FLASH_init()
{
	PORTC.OUTCLR = PIN2_bm;
	_delay_us(1);
	SPIC_Write(Flash_EWSR); //Enable status write
	PORTC.OUTSET = PIN2_bm;
	_delay_us(30);
	PORTC.OUTCLR = PIN2_bm;
	_delay_us(1);
	SPIC_Write(Flash_WRSR); //Adresse 0x00
	SPIC_Write(0); //Adresse 0x00
	PORTC.OUTSET = PIN2_bm;
}

void Flash_block_erase(uint32_t Address)
{
	SS_Port.OUTCLR = SS_Pin;
	SPIC_Write(Block_erase); //Adresse 0x00
	SPIC_Write((uint8_t)(Address>>16)); //Adresse 0x00
	SPIC_Write((uint8_t)(Address>>8)); //Adresse 0x00
	SPIC_Write((uint8_t)(Address)); //Adresse 0x00
	SS_Port.OUTSET = SS_Pin;
	//_delay_ms(30);
}

void Flash_start_write_AAI(uint32_t Address, unsigned char c)
{
    
	SS_Port.OUTCLR = SS_Pin;
	SPIC_Write(AAI_Write); //Adresse 0x00
	SPIC_Write((uint8_t)(Address>>16)); //Adresse 0x00
	SPIC_Write((uint8_t)(Address>>8)); //Adresse 0x00
	SPIC_Write((uint8_t)(Address)); //Adresse 0x00
	SPIC_Write(c);
	SS_Port.OUTSET = SS_Pin;
	_delay_us(25);
	//for(uint8_t i=1; i<length; i++)
	//{
		//SS_Port.OUTCLR = SS_Pin;
		//SPIC_Write(AAI_Write); //Adresse 0x00
		//SPIC_Write(*c++); //Adresse 0x00
		//SS_Port.OUTSET = SS_Pin;
		//position++;
		//_delay_us(25);
	//}
	//SS_Port.OUTSET = SS_Pin;
}

void Flash_write_AAI( unsigned char c)
{
	SS_Port.OUTCLR = SS_Pin;
	SPIC_Write(AAI_Write); //Adresse 0x00
	SPIC_Write(c); //Adresse 0x00
	SS_Port.OUTSET = SS_Pin;
	_delay_us(25);
	//for(uint8_t i=0; i<length; i++)
	//{
		//SS_Port.OUTCLR = SS_Pin;
		//SPIC_Write(AAI_Write); //Adresse 0x00
		//SPIC_Write(*c++); //Adresse 0x00
		//SS_Port.OUTSET = SS_Pin;
		//_delay_us(25);
	//}
	//SS_Port.OUTSET = SS_Pin;
}

void Flash_set_WREN()
{
	SS_Port.OUTCLR = SS_Pin;
	SPIC_Write(Flash_WREN); //Adresse 0x00
	SS_Port.OUTSET = SS_Pin;
}

void Flash_set_WRDI()
{
	SS_Port.OUTCLR = SS_Pin;
	SPIC_Write(Flash_WRDI); //Adresse 0x00
	SS_Port.OUTSET = SS_Pin;
}