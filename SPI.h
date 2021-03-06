/*
 * SPI.h
 *
 * Created: 07.05.2017 14:43:24
 *  Author: JackFrost
 */ 


#ifndef SPI_H_
#define SPI_H_

#include "CAN.h"

void SPIC_Init(void);
unsigned char SPIC_Read_Write(unsigned char data);
void SPIC_Write(unsigned char data);
void SPIC_Write_array(uint8_t *data, uint8_t length);
void SPIC_Read_array(uint8_t *data, uint8_t length);
void SPIC_Read_RX_buffer(RX_CAN_t *data);
void SPIC_write_tx_buffer(RX_CAN_t *data);


#endif /* SPI_H_ */