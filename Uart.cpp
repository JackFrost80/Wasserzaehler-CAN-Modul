/*
 * Uart.cpp
 *
 * Created: 18.06.2017 18:06:00
 *  Author: JackFrost
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>

void Send_UART_NNL_D0(char data[])
{
	unsigned char Counter = 0x00;
	unsigned char lenght = 0x00;

	lenght = strlen(data);

	while(Counter < lenght)
	{
		while (!(USARTD0.STATUS & USART_DREIF_bm));
		USARTD0.DATA = data[Counter];
		Counter++;
	}

	Counter = 0x00;
	
}

void Send_UART_D0(char data[])
{
	unsigned char Counter = 0x00;
	unsigned char lenght = 0x00;

	lenght = strlen(data);

	while(Counter < lenght)
	{
		while (!(USARTD0.STATUS & USART_DREIF_bm));
		USARTD0.DATA = data[Counter];
		Counter++;
	}

	Counter = 0x00;
	while (!( USARTD0.STATUS & USART_DREIF_bm));
	USARTD0.DATA = 0x0A;
	while (!( USARTD0.STATUS & USART_DREIF_bm));
	USARTD0.DATA = 0x0D;
}
