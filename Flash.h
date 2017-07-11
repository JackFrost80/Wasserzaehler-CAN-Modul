/*
 * Flash.h
 *
 * Created: 18.06.2017 16:12:45
 *  Author: JackFrost
 */ 


#ifndef FLASH_H_
#define FLASH_H_

#define SS_Port PORTC
#define SS_Pin PIN2_bm

#define Sector_erase 0x20
#define Block_erase 0x52
#define Chip_erase 0x60
#define AAI_Write 0xAF
#define Byte_write 0x02
#define Flash_Read 0x03
#define Flash_WREN 0x06
#define Flash_WRDI 0x04
#define Flash_WRSR 0x01
#define Flash_EWSR 0x50
#define Flash_RDSR 0x05

void FLASH_init();
void Flash_block_erase(uint32_t Address);
void Flash_start_write_AAI(uint32_t Address,unsigned char *c,uint8_t length);
void Flash_write_AAI(unsigned char *c,uint8_t length);
void Flash_set_WREN();
void Flash_set_WRDI();

#endif /* FLASH_H_ */