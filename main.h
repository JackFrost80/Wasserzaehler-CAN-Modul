/*
 * main.h
 *
 * Created: 16.05.2017 21:11:48
 *  Author: JackFrost
 */ 


#ifndef MAIN_H_
#define MAIN_H_


#define KEY_PORT_A        PORTD
#define KEYS				1
#define KEY_PIN			 PORTD.IN
#define KEY0	            1
#define ALL_KEYS        (1<<KEY0)

#define REPEAT_MASK     ( 1<<KEY0  )       // repeat: key1, key2
#define REPEAT_START    50                        // after 500ms
#define REPEAT_NEXT     20                        // every 200ms



#endif /* MAIN_H_ */