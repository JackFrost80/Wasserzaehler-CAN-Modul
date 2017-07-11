/*
 * CAN.h
 *
 * Created: 07.05.2017 14:43:42
 *  Author: JackFrost
 */ 


#ifndef CAN_H_
#define CAN_H_

#define System_Message 0x0001
#define Bootloader_Message 0x080F
#define Water_message 0x03F0 

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



#endif /* CAN_H_ */