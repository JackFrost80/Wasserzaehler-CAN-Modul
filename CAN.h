/*
 * CAN.h
 *
 * Created: 07.05.2017 14:43:42
 *  Author: JackFrost
 */ 

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef CAN_H_
#define CAN_H_

#define System_Message 0x0001
#define Start_Firmware 0x070F
#define Firmware 0x0710
#define End_Firmware 0x0711
#define Bootloader_Message 0x0712
#define Send_Firmware 0x0713
#define Water_message 0x03F0 
#define Water_flow 0x03F1

typedef struct RX_CAN {
	
	uint16_t ID;
	bool Remote_frame;
	uint8_t length;
	uint8_t data[8];
} RX_CAN_t, *p_RX_can_t;


typedef enum System_message_type
{
	midnight_gc = 0x00,  /* 1x Tq Sync */
	midnight_clock_gc = 0x01,  /* 2x Tq Syn */
	clock_gc = 0x02,  /* 3x Tq Syn */
	new_minute_gc = 0x03,  /* 4x Tq Syn */
	new_hour_gc = 0x04,  /* 4x Tq Syn */
} System_message_type_t;

void CAN_init();
void Fill_CAN_Buffer(uint8_t * _Buffer);
bool read_can_tx_status(uint8_t buffer);

#endif /* CAN_H_ */