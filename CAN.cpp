/*
 * CAN.cpp
 *
 * Created: 07.05.2017 14:43:34
 *  Author: JackFrost
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include "CAN.h"
#include "MCP2515.h"
#include "SPI.h"

void CAN_init()
{
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(reset_command);
	_delay_us(0.08);
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(read_command);
	SPIC_Write(CANCTRL);
		PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(write_command);
	SPIC_Write(CNF3);
	SPIC_Write(0x05);  //PHSEG2 set to 6x Tq
	SPIC_Write(BTLMODE_bm | (0x06<<PHSEG1_bp) | 0x01);  //PHSEG1 7x Tq | PropSec 2x Tq
	SPIC_Write(one_time_quanta_gc | 0x01);  // Prescaler 4 and 1x Tq sync
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(write_command);
	SPIC_Write(RXM0SIDH);
	SPIC_Write(0x80);  //Mask Bit 11 RX0
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(write_command);
	SPIC_Write(RXM1SIDH);
	SPIC_Write(0x80);  //Mask Bit 11 RX1
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(write_command);
	SPIC_Write(RXF2SIDH);
	SPIC_Write(0x80);  //Filter Bit 11 RX1
	SPIC_Write(0xE0);  //Filter Bit 11 RX1
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	
	PORTA.OUTCLR = PIN1_bm;  //Filter 0 
	_delay_us(0.08);
	SPIC_Write(write_command);
	SPIC_Write(RXF0SIDH);
	SPIC_Write(0x00);  //Filter Bit 11 RX1
	SPIC_Write(0xE0);  //Filter Bit 11 RX1
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	
	PORTA.OUTCLR = PIN1_bm;  //Filter 0 
	_delay_us(0.08);
	SPIC_Write(write_command);
	SPIC_Write(RXF1SIDH);
	SPIC_Write(0x00);  //Filter Bit 11 RX1
	SPIC_Write(0xE0);  //Filter Bit 11 RX1
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	
	PORTA.OUTCLR = PIN1_bm;  //Filter 0 
	_delay_us(0.08);
	SPIC_Write(write_command);
	SPIC_Write(RXF3SIDH);
	SPIC_Write(0x80);  //Filter Bit 11 RX1
	SPIC_Write(0xE0);  //Filter Bit 11 RX1
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	
	PORTA.OUTCLR = PIN1_bm;  //Filter 0 
	_delay_us(0.08);
	SPIC_Write(write_command);
	SPIC_Write(RXF4SIDH);
	SPIC_Write(0x80);  //Filter Bit 11 RX1
	SPIC_Write(0xE0);  //Filter Bit 11 RX1
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	
	PORTA.OUTCLR = PIN1_bm;  //Filter 0 
	_delay_us(0.08);
	SPIC_Write(write_command);
	SPIC_Write(RXF5SIDH);
	SPIC_Write(0x80);  //Filter Bit 11 RX1
	SPIC_Write(0xE0);  //Filter Bit 11 RX1
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	
	
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(read_command);
	SPIC_Write(CNF3);
	SPIC_Read_Write(0xFF);
	SPIC_Read_Write(0xFF);
	SPIC_Read_Write(0xFF);
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(read_command);
	SPIC_Write(CANCTRL);
	SPIC_Read_Write(0xFF);
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(read_command);
	SPIC_Write(RXM0SIDH);
	SPIC_Write(0x81);  //Mask Bit 11 RX0
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(read_command);
	SPIC_Write(RXM1SIDH);
	SPIC_Write(0x80);  //Mask Bit 11 RX1
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(read_command);
	SPIC_Write(RXF2SIDH);
	SPIC_Write(0x80);  //Filter Bit 11 RX1
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(read_command);
	SPIC_Write(RXF0SIDH);
	SPIC_Write(0x80);  //Filter Bit 11 RX1
	PORTA.OUTSET = PIN1_bm;
	_delay_us(10);
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(write_command);
	SPIC_Write(CANCTRL);
	SPIC_Write(OSM_bm | normal_operation_mode_gc | CLKEN_bm | eighth_system_clock_gc);
	PORTA.OUTSET = PIN1_bm;
	
	

}

void Fill_CAN_Buffer(uint8_t * _Buffer)
{
	_Buffer[0] = Transmit_Buffer_Priority_MED_gc;
	_Buffer[1] = (uint8_t)(Water_message >>3);
	_Buffer[2] = (uint8_t)(Water_message <<5);
	_Buffer[3] = 0x00;
	_Buffer[4] = 0x00;
	
	
	
}

bool read_can_tx_status(uint8_t tx_buffer)
{
	bool status = false;
	PORTA.OUTCLR = PIN1_bm;
	_delay_us(0.08);
	SPIC_Write(read_command);
	switch(tx_buffer)
	{
		case 0:
		{
			SPIC_Write(TXB0CTRL);
			
		}
		break;
		case 1:
		{
			SPIC_Write(TXB1CTRL);
			
		}
		break;
		case 2:
		{
			SPIC_Write(TXB2CTRL);
			
		}
		break;
		
		default :
		return false;
		
	
	}
	uint8_t helper = SPIC_Read_Write(0xFF);
	 if(!(helper & TXREQ_bm))
		status = true;
	PORTA.OUTSET = PIN1_bm;
	return status;
}