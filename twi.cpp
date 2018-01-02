/*
 * twi.cpp
 *
 * Created: 14.05.2017 21:10:33
 *  Author: JackFrost
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/delay.h>


void twi_write(TWI_t *twiname, uint8_t *writeData,uint8_t page_adress,uint8_t Adress, uint8_t bytes,bool fixed){

	uint8_t i;
	TWIC_MASTER_CTRLC &= ~((1<<TWI_MASTER_ACKACT_bp));
	twiname->MASTER.ADDR = page_adress;
	while (!(TWIC.MASTER.STATUS & TWI_MASTER_WIF_bm));
	if(TWIC_MASTER_STATUS & (1<<TWI_MASTER_ARBLOST_bp))
	{
		twiname->MASTER.CTRLA = 0;
		PORTC.DIRSET = PIN1_bm;
		for(uint8_t i=0;i<9;i++)
		{
			PORTC.OUTSET = PIN1_bm;
			_delay_us(20);
			PORTC.OUTCLR = PIN1_bm;
			_delay_us(20);
		}
		PORTC.DIRCLR = PIN1_bm;
		twiname->MASTER.CTRLA = TWI_MASTER_ENABLE_bm;
		twiname->MASTER.STATUS = TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm;
		twiname->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
		TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		twiname->MASTER.ADDR = page_adress;
		if(TWIC_MASTER_STATUS & (1<<TWI_MASTER_ARBLOST_bp));
		
	}
	twiname->MASTER.DATA = Adress;       // write word addr
	while(!(twiname->MASTER.STATUS&TWI_MASTER_WIF_bm));
	for(i=0;i<bytes;i++){
		if(!fixed)             // write data 
		twiname->MASTER.DATA =writeData[i];
		else
		twiname->MASTER.DATA =writeData[0];
		while(!(twiname->MASTER.STATUS&TWI_MASTER_WIF_bm));
	}
	TWIC.MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
}

void twi_read(TWI_t *twiname, uint8_t *readData,uint8_t page_adress, uint8_t Adress, uint8_t bytes)
{
	twiname->MASTER.CTRLC &= ~((1<<TWI_MASTER_ACKACT_bp));
	twiname->MASTER.ADDR = page_adress;
	while (!(twiname->MASTER.STATUS & TWI_MASTER_WIF_bm));
	if(twiname->MASTER.STATUS & (1<<TWI_MASTER_ARBLOST_bp))
	{
		twiname->MASTER.CTRLA = 0;
		PORTC.DIRSET = PIN1_bm;
		for(uint8_t i=0;i<9;i++)
		{
			PORTC.OUTSET = PIN1_bm;
			_delay_us(20);
			PORTC.OUTCLR = PIN1_bm;
			_delay_us(20);
		}
		PORTC.DIRCLR = PIN1_bm;
		twiname->MASTER.CTRLA = TWI_MASTER_ENABLE_bm;
		twiname->MASTER.STATUS = TWI_MASTER_ARBLOST_bm | TWI_MASTER_BUSERR_bm;
		twiname->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;
		twiname->MASTER.CTRLC = TWI_MASTER_CMD_STOP_gc;
		twiname->MASTER.ADDR = page_adress;
		while (!(twiname->MASTER.STATUS & TWI_MASTER_WIF_bm));
	}
	twiname->MASTER.DATA = Adress;
	while (!(twiname->MASTER.STATUS & TWI_MASTER_WIF_bm));
	page_adress |= 0x01;
	twiname->MASTER.ADDR = page_adress;
	for(volatile uint8_t j=0 ;j<bytes ;j++){
		
		while(!(twiname->MASTER.STATUS&TWI_MASTER_RIF_bm));
		if(j == (bytes-1))
		{
			twiname->MASTER.CTRLC = (1<<TWI_MASTER_ACKACT_bp) | TWI_MASTER_CMD_STOP_gc;
		}
		readData[j] = twiname->MASTER.DATA;
	}
}


