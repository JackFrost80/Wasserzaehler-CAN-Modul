/*
 * Wasserzaehler.cpp
 *
 * Created: 02.05.2017 21:21:05
 * Author : JackFrost
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include "SPI.h"
#include "CAN.h"
#include "MCP2515.h"
#include "twi.h"
#include "main.h"
#include "Flash.h"
#include "Uart.h"


extern "C"
{
	#include "time_convert.h"
	
}

#define DMA_TX_ESP_CHANNEL	EDMA.CH2
#define USART_Netzwerk		USARTD0
#define CLK_Prescaler 0x00
#define PLL_Faktor 16
#define BSEL 3325
#define BSCALE 0xFD
#define CPU_SPEED 32000000UL
#define BAUDRATE    100000UL
#define TWI_BAUD(F_SYS, F_TWI) ((F_SYS / (2 * F_TWI)) - 5)
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)
#define FRAM_base_adress 0xA0
#define ZeitZone CET+info->tm_isdst



volatile uint8_t key_state;
volatile uint8_t key_press;
volatile uint8_t key_rpt;                                  // key long press and repeat
volatile uint32_t zaehlerstand = 218638;
time_t Unixtimestamp = 1483229369;
time_t Unixtimestamp_old = 1483229369;
uint32_t zaehlerstand_EEPROM EEMEM = 218638;
uint16_t Verbrauch_heute_EEPROM EEMEM = 0;
uint16_t Verbrauch_gestern_EEPROM EEMEM = 0;
uint16_t Verbrauch_heute = 0;
uint16_t Verbrauch_gestern = 0;
uint32_t Zaehlerstand_alt = 218368;
char send_buffer[96] = {0};
volatile uint16_t counter = 0;
volatile uint8_t number_chars = 0;
volatile bool ESP_complete = false;
volatile bool start = false;
char int_buffer[30] = {0};
char ESP_buffer[30] = {0};
const void * tx_buffer_Pointer = &send_buffer[0];
tm *info;
bool set_time = false;
char Buffer[32] = {0};
volatile bool force_summertime_check = false;
volatile bool data_changed = false;
uint8_t CAN_Data[14] = {0};
bool CAN_send = false;
uint8_t can_status = 0;
uint8_t can_rx_status = 0;
uint8_t can_status_recieved = 0;
uint8_t can_error = 0;
volatile bool can_control = true;
bool New_RX0 = false;
bool New_RX1 = false;
uint8_t buffer_FRAM[6] = {0};
unsigned char firmware[256] = {0};
volatile bool update = false;
volatile bool first_byte = false;
volatile bool last_byte = false;
volatile bool firmware_ready = false;
volatile bool Wasser = false;
RX_CAN_t rx_can0;
RX_CAN_t rx_can1;
RX_CAN_t tx_can0;
RX_CAN_t tx_can1;
p_RX_can_t p_rx_can0 = &rx_can0;
p_RX_can_t p_rx_can1 = &rx_can1;
p_RX_can_t p_tx_can0 = &tx_can0;
p_RX_can_t p_tx_can1 = &tx_can1;


ISR( TCC5_OVF_vect )                            // every 10ms
{
	static uint8_t ct0 = 0xFF, ct1 = 0xFF, rpt;
	uint8_t i;
	
	//TCC0_CNT = 64285;  // ~8,7 ms bei 30 MHz und Prescaler 1024
	i = key_state ^ PORTD.IN;// key changed ?
	ct0 = ~( ct0 & i );// reset or count ct0
	ct1 = ct0 ^ (ct1 & i);
	i &= ct0 & ct1;
	
	key_state ^= i;
	key_press |= key_state & i;                     // 0->1: key press detect
	if( (key_state & REPEAT_MASK) == 0 )            // check repeat function
	rpt = REPEAT_START;
	// start delay
	if( --rpt == 0 ){
		rpt = REPEAT_NEXT;                            // repeat delay
		key_rpt = key_state ;
	}
}


ISR(TCC4_CCB_vect)
{
	
	uint8_t helper = (uint8_t) (TCC4.CCB>>8);
	if((helper > 0x5C) && (helper < 0x64) )
	{
		PORTD.OUTTGL = PIN0_bm;
		zaehlerstand++;
		data_changed =  true;
		
	}
	TCC4.INTFLAGS = TC4_CCBIF_bm;
}

ISR(RTC_OVF_vect)
{
	if(!set_time)
	Unixtimestamp++;
}

ISR(EDMA_CH2_vect, ISR_BLOCK)
{
	//PORTE.OUTTGL = PIN2_bm;
	DMA_TX_ESP_CHANNEL.TRFCNT = 0;
	DMA_TX_ESP_CHANNEL.CTRLA &= ~EDMA_CH_ENABLE_bm;  // this shouldn't be necessary
	DMA_TX_ESP_CHANNEL.CTRLB |= (EDMA_CH_TRNIF_bm | EDMA_CH_ERRIF_bm);
	memset(send_buffer, '\0', sizeof(send_buffer));
}

ISR(USARTD0_RXC_vect)
{
	//if(USARTC0.STATUS & USART_FERR_bm)
	//Frame_error++;
	//if(USARTC0.STATUS & USART_PERR_bm)
	//Parity_error++;
	unsigned char data = USART_Netzwerk.DATA;
	static unsigned char helper = 0;
	if(start)
	{
		if(!update)
		{
			if(counter == 0)
			{
				number_chars = data - 'a';
				counter++;
			}
			else
			{
				ESP_buffer[counter-1] = data;
				counter++;
			}
			if(data == 0x7F)
			{
				start = false;
				ESP_complete = true;
			}
		}
		else
		{
			if(data != 0x7F)
			{
				firmware[counter] = data;
				counter++;
			}
			else
			{
				start = false;
				firmware_ready = true;
			}

		}

	}
	if(data=='a' && !start)
	{
		start = true;
		counter= 0;
	}
	else
	{
		if(data=='f' && !start)
		{
			first_byte = true;
			start = true;
			counter=0;
			update = true;
		}
		if(data=='g' && !start)
		{
			start = true;
			counter=0;
			update = true;
		}
		if(data=='e' && !start)
		{
			update = false;
		}
	}
}

uint8_t get_key_state( uint8_t key_mask)
{
	key_mask &= key_state;
	return key_mask;
}

void twi_init(TWI_t * twiname){

	PORTC.PIN0CTRL = PORT_OPC_WIREDAND_gc;
	PORTC.PIN1CTRL = PORT_OPC_WIREDAND_gc;
	twiname->MASTER.CTRLB = TWI_MASTER_SMEN_bm;
	twiname->MASTER.BAUD = TWI_BAUDSETTING;
	twiname->MASTER.CTRLA = TWI_MASTER_ENABLE_bm;
	twiname->MASTER.STATUS = TWI_MASTER_BUSSTATE_IDLE_gc;


	return;
}

void DMA_TX_ESP_init()
{
	// reset source address
	DMA_TX_ESP_CHANNEL.ADDRL = 0;
	DMA_TX_ESP_CHANNEL.ADDRH = 0;

	// set up destination address nicht nötig im Peripheral mode
	//DMA_TX_ESP_CHANNEL.DESTADDRL = (((uint16_t) &USART_Netzwerk.DATA) >> 0) & 0xff;
	//DMA_TX_ESP_CHANNEL.DESTADDRH = (((uint16_t) &USART_Netzwerk.DATA) >> 8) & 0xff;
	
	DMA_TX_ESP_CHANNEL.ADDRCTRL |= EDMA_CH_DIR_INC_gc;        // increment source address during transfer
	DMA_TX_ESP_CHANNEL.ADDRCTRL |= EDMA_CH_DESTRELOAD_NONE_gc;      // destination address does not need to be reloaded
	DMA_TX_ESP_CHANNEL.ADDRCTRL |= EDMA_CH_DESTDIR_INC_gc;        // Memory address incremented

	DMA_TX_ESP_CHANNEL.TRIGSRC = EDMA_CH_TRIGSRC_USARTD0_DRE_gc;      // automatically trigger a new burst when UARTD0 is ready
	DMA_TX_ESP_CHANNEL.TRFCNT = 0;                    // reset block size
	DMA_TX_ESP_CHANNEL.CTRLA = EDMA_CH_SINGLE_bm;            // single shot mode (i.e. one burst transfer per trigger event)
	DMA_TX_ESP_CHANNEL.CTRLB |= EDMA_CH_TRNINTLVL_HI_gc;
}

void RTC_Init(void)
{
	//Interner 32KHz Oszi
	CLK.RTCCTRL = CLK_RTCSRC_RCOSC_gc | CLK_RTCEN_bm;
	//0,5s Interrupt
	RTC.PER = 1023;
	//Prescale 1
	RTC.CTRL = RTC_PRESCALER_DIV1_gc;
	//Synchronisieren
	//while(OSC.STATUS & RTC_SYNCBUSY_bm);
	RTC.INTCTRL = 0x03;
	
}

void ESP_init()
{
	PORTD.PIN5CTRL = PORT_OPC_WIREDANDPULL_gc;
	PORTD.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;
}

void Clock_Init(void)
{
	//Interner 2MHz Oszi
	OSC.XOSCCTRL =  OSC_X32KLPM_bm | OSC_XOSCSEL1_bm;  // 32 kHz Quarz settings.
	OSC.CTRL = OSC_RC32MEN_bm  | OSC_XOSCEN_bm;
	// Warten bis Oszillator stabil ist
	while ((OSC.STATUS & OSC_RC32MEN_bm) == 0);
	//while ((OSC.STATUS & OSC_XOSCEN_bm) == 0);
	
	// I/O Protection
	CCP = CCP_IOREG_gc;
	// Prescaler
	//CLK.PSCTRL = CLK_Prescaler;
	// PLL Sorce und PLL Faktor
	//OSC.PLLCTRL = OSC_PLLSRC_RC2M_gc | (PLL_Faktor << OSC_PLLFAC_gp);
	// PLL enable
	//OSC.CTRL = OSC_PLLEN_bm ;
	//while ((OSC.STATUS & OSC_PLLRDY_bm) == 0);
	// I/O Protection
	//CCP = CCP_IOREG_gc;
	// System Clock selection
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
	// DFLL ein (Auto Kalibrierung)
	//DFLLRC32M.CTRL = DFLL_ENABLE_bm;
	
}

void Timer_init(void)
{
	EVSYS.CH0MUX = EVSYS_CHMUX_PORTA_PIN0_gc;
	PORTA.PIN0CTRL |= PORT_ISC_BOTHEDGES_gc;
	
	TCC4.CTRLE = TC_HCCBMODE_CAPT_gc;
	TCC4.CTRLD =  TC_EVACT_PWF_gc | TC_EVSEL_CH0_gc ; // Pulese detection and Event channel 0
	
	TCC4.INTCTRLB = TC4_CCBINTLVL1_bm | TC4_CCBINTLVL0_bm;
	TCC4.CTRLA = TC_CLKSEL_DIV2_gc;
	TCC5.INTCTRLA = TC_OVFINTLVL_MED_gc; // Debounce timer
	TCC5.PER = 39999;
	TCC5.CTRLA = TC_CLKSEL_DIV8_gc;
	

}

void memset_volatile(volatile unsigned char *s, char c, size_t n)
{
	volatile unsigned char *p = s;
	while (n-- > 0) {
		*p++ = c;
	}
}



int main(void)
{
	bool cleared = false;
	bool ready_to_write = false;
	USARTD0.CTRLA |= USART_RXCINTLVL_HI_gc;
	USARTD0.CTRLB |= USART_RXEN_bm | USART_TXEN_bm;
	USARTD0.CTRLC |= USART_PMODE_EVEN_gc | USART_SBMODE_bm | USART_CHSIZE_8BIT_gc;
	USARTD0.BAUDCTRLA = (uint8_t)BSEL;
	USARTD0.BAUDCTRLB = (uint8_t)(BSEL>>8) | (uint8_t)(BSCALE << 4);
	bool saved = false;
	bool EEPROM_Warning = false;
	bool neue_Stunde = false;
	volatile bool send_answer;
	volatile char respond = 0;
	//zaehlerstand = eeprom_read_dword(&zaehlerstand_EEPROM);
	
	Clock_Init();
	RTC_Init();
	DMA_TX_ESP_init();
	twi_init(&TWIC);
	ESP_init();
	EDMA.CTRL |= EDMA_ENABLE_bm;
	PORTD.DIRSET = PIN3_bm;		// PC3 (TXD0) as output
	PORTD.DIRCLR = PIN2_bm;		// PC2 (RXD0) as input
	PORTD.PIN2CTRL = PORT_OPC_PULLUP_gc;
	PORTA.DIRSET = PIN7_bm;  //LED
	PORTA.PIN0CTRL = PORT_OPC_PULLDOWN_gc;
	PORTD.DIRSET = PIN0_bm;  //LED
	//PORTC.PIN3CTRL = PORT_OPC_PULLUP_gc;
	_delay_ms(10);
	twi_read(&TWIC,buffer_FRAM,FRAM_base_adress,0,6);
	zaehlerstand = (uint32_t)buffer_FRAM[0] <<24 | (uint32_t)buffer_FRAM[1] <<16 | (uint32_t)buffer_FRAM[2] << 8 | (uint32_t)buffer_FRAM[3];
	Verbrauch_heute = (uint32_t)buffer_FRAM[4] << 8 | (uint32_t)buffer_FRAM[5];
	Zaehlerstand_alt = zaehlerstand;
	info = gurke(&Unixtimestamp,CET);
	SPIC_Init();
	CAN_init();
	FLASH_init();
	PMIC.CTRL |= PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;
	sei();
	Timer_init();
	Fill_CAN_Buffer(CAN_Data);
	while (1)
	{
		
		PORTA.OUTCLR = PIN1_bm;
		_delay_us(0.08);
		SPIC_Write(read_status);
		can_status = SPIC_Read_Write(0xFF);
		PORTA.OUTSET = PIN1_bm;
		_delay_us(1);
		PORTA.OUTCLR = PIN1_bm;
		_delay_us(0.08);
		SPIC_Write(RX_status);
		can_rx_status = SPIC_Read_Write(0xFF);
		PORTA.OUTSET = PIN1_bm;
		PORTA.OUTCLR = PIN1_bm;
		_delay_us(0.08);
		SPIC_Write(read_command);
		SPIC_Write(EFLG);
		can_error = SPIC_Read_Write(0xFF);
		PORTA.OUTSET = PIN1_bm;
		
		//if(get_key_state(1<<KEY0))
		//{
			//if(can_error && 0x20)
			//{
				//PORTA.OUTSET = PIN7_bm;
				//PORTD.OUTCLR = PIN0_bm;
			//}
			//else
			//{
				//PORTA.OUTCLR = PIN7_bm;
				//PORTD.OUTSET = PIN0_bm;
			//}
			//
		//}
		//else
		//{
			//if(can_error && 0x18)
			//{
				//PORTA.OUTSET = PIN7_bm;
				//PORTD.OUTCLR = PIN0_bm;
			//}
			//else
			//{
				//if(can_error && 0x06)
				//{
					//PORTA.OUTSET = PIN7_bm;
					//PORTD.OUTSET = PIN0_bm;
				//}
				//else
				//{
					//PORTA.OUTCLR = PIN7_bm;
					//PORTD.OUTSET = PIN0_bm;
				//}
			//}
			//
		//}
		
		if(data_changed)
		{
			data_changed = false;
			cli();
			Verbrauch_heute += zaehlerstand - Zaehlerstand_alt;
			Zaehlerstand_alt = zaehlerstand;
			sei();
			
			buffer_FRAM[0] = (uint8_t) (Zaehlerstand_alt>>24);
			buffer_FRAM[1] = (uint8_t) (Zaehlerstand_alt>>16);
			buffer_FRAM[2] = (uint8_t) (Zaehlerstand_alt>>8);
			buffer_FRAM[3] = (uint8_t) Zaehlerstand_alt;
			buffer_FRAM[4] = (uint8_t) (Verbrauch_heute>>8);
			buffer_FRAM[5] = (uint8_t) Verbrauch_heute;
			twi_write(&TWIC,buffer_FRAM,FRAM_base_adress,0,6,false);
			Wasser = true;
			
		}
		if(Unixtimestamp != Unixtimestamp_old)
		{
			Unixtimestamp_old = Unixtimestamp;
			info = gurke(&Unixtimestamp,CET);
		}
		
		if((can_rx_status && 0xB0) != 0)
		{
			
			PORTA.OUTTGL = PIN7_bm;
			if((can_rx_status && 0x40) != 0)
			{
				PORTA.OUTCLR = PIN1_bm;
				_delay_us(0.08);
				SPIC_Write(Read_RX0_SID);
				
				PORTA.OUTSET = PIN1_bm;
				New_RX0 = true;
			}
			if((can_rx_status && 0x80) != 0)
			{
				PORTA.OUTCLR = PIN1_bm;
				_delay_us(0.08);
				SPIC_Write(Read_RX1_SID);
				SPIC_Read_RX_buffer(p_rx_can1);
				PORTA.OUTSET = PIN1_bm;
				New_RX1 = true;
			}
		}
		

		
		if(New_RX0)
		{
			New_RX0 = false;
			switch (p_rx_can0->ID)
			{
				case System_Message:      //System message processing
				{
					can_status_recieved = p_rx_can0->data[0];
					if(can_status_recieved  == clock_gc )
					{
						cli();
						Unixtimestamp = (uint32_t)p_rx_can0->data[1]<<24 | (uint32_t)p_rx_can0->data[2]<<16 | (uint32_t)p_rx_can0->data[3]<<8 | (uint32_t)p_rx_can0->data[4];
						sei();
						info = gurke(&Unixtimestamp,CET);
						force_summertime_check = true;
						
					}
					
					if(can_control)
					{
						
						if(can_status_recieved  == midnight_clock_gc )
						{
							can_status_recieved &= ~(0x01);
							Verbrauch_gestern = Verbrauch_heute;
							Verbrauch_heute = 0;
							cli();
							Unixtimestamp = (uint32_t)p_rx_can0->data[1]<<24 | (uint32_t)p_rx_can0->data[2]<<16 | (uint32_t)p_rx_can0->data[3]<<8 | (uint32_t)p_rx_can0->data[4];
							sei();
							info = gurke(&Unixtimestamp,CET);
							force_summertime_check = true;
						}
						if(can_status_recieved  == midnight_gc )
						{
							can_status_recieved &= ~(0x01);
							Verbrauch_gestern = Verbrauch_heute;
							Verbrauch_heute = 0;
						}
					}
				}
				break; //  Ende Systemmessage
				
				case Start_Firmware:
				{   
					first_byte = true; // Start delete old Firmware in Flashchip
					cleared = false;
					ready_to_write = false;
					memcpy(p_rx_can0->data,&firmware[0],8); // Copy Data from Buffer to firmware buffer
					counter += 8;
					
				}
				break; // Ende Firmwware start
				case Firmware:
				{
					memcpy(p_rx_can0->data,&firmware[counter],8); // Copy Data from Buffer to firmware buffer
					counter += 8;
					if(counter <= 248)
					{
						p_tx_can1->ID = Firmware;
						p_tx_can1->Remote_frame = true;
						p_tx_can1->length = 0;
						PORTA.OUTCLR = PIN1_bm;
						_delay_us(0.08);
						SPIC_Write(write_command);
						SPIC_Write(TXB1CTRL);
						SPIC_Write(0x00);
						SPIC_write_tx_buffer(p_tx_can1);
						PORTA.OUTSET = PIN1_bm;
						_delay_us(10);
						PORTA.OUTCLR = PIN1_bm;
						_delay_us(0.08);
						SPIC_Write(send_buffer_1);
						PORTA.OUTSET = PIN1_bm;  // Request new Firmwareframe
					}
					else
						firmware_ready = true;
						
					
				}
				break;
				
				default:
				break;
			}
		}
		
		if(!can_control)	
		{	
			if((info->tm_hour == 0) && (info->tm_min == 0)  && (info->tm_sec <= 20) && !neue_Stunde && !set_time)
			{
			
				neue_Stunde = true;
				Verbrauch_gestern = Verbrauch_heute;
				Verbrauch_heute = 0;
			}
		}
		if((info->tm_min == 59) && (!EEPROM_Warning))
		{
			// LED Schalten !!
			EEPROM_Warning = true;
		}
		if((info->tm_min == 0)  && (!saved) && !set_time)
		{
			summertime(info);
			//eeprom_write_dword(&zaehlerstand_EEPROM,zaehlerstand);
			//eeprom_write_word(&Verbrauch_heute_EEPROM,Verbrauch_heute);
			//eeprom_write_word(&Verbrauch_gestern_EEPROM,Verbrauch_gestern);
		}
		if((info->tm_min == 1)  && (saved))
		saved=false;
		
		if((info->tm_sec <= 05)  && (!CAN_send))
		{
			CAN_send = true;
			p_tx_can0->ID = Water_message;
			p_tx_can0->length = 0x06;  // 6 Bytes
			p_tx_can0->data[0] = (uint8_t) (Zaehlerstand_alt >> 24);
			p_tx_can0->data[1] = (uint8_t) (Zaehlerstand_alt >> 16);
			p_tx_can0->data[2] = (uint8_t) (Zaehlerstand_alt >> 8);
			p_tx_can0->data[3] = (uint8_t) (Zaehlerstand_alt);
			p_tx_can0->data[4] = (uint8_t) (Verbrauch_heute >> 8);
			p_tx_can0->data[5] = (uint8_t) (Verbrauch_heute);
			p_tx_can0->Remote_frame = false;
			//CAN_Data[5] = 0x06;  // 6 Bytes
			//CAN_Data[6] = (uint8_t) (Zaehlerstand_alt >> 24);
			//CAN_Data[7] = (uint8_t) (Zaehlerstand_alt >> 16);
			//CAN_Data[8] = (uint8_t) (Zaehlerstand_alt >> 8);
			//CAN_Data[9] = (uint8_t) (Zaehlerstand_alt);
			//CAN_Data[10] = (uint8_t) (Verbrauch_heute >> 8);
			//CAN_Data[11] = (uint8_t) (Verbrauch_heute);
			PORTA.OUTCLR = PIN1_bm;
			_delay_us(0.08);
			SPIC_Write(write_command);
			SPIC_Write(TXB0CTRL);
			SPIC_Write(0x2);  // Mid High Message;
			SPIC_write_tx_buffer(p_tx_can0);
			//SPIC_Write_array(CAN_Data,12);
			PORTA.OUTSET = PIN1_bm;
			_delay_us(10);
			PORTA.OUTCLR = PIN1_bm;
			_delay_us(0.08);
			SPIC_Write(send_buffer_0);
			PORTA.OUTSET = PIN1_bm;
			
		}
		
		if(Wasser)
		{
			Wasser = false;
			p_tx_can1->ID = Water_flow;
			p_tx_can1->length = 0x01;  // 1 Byte
			p_tx_can1->data[0] = 0x01;
			p_tx_can1->Remote_frame = false;
			//CAN_Data[1] = (uint8_t)(Water_flow >>3);
			//CAN_Data[2] = (uint8_t)(Water_flow <<5);
			//CAN_Data[5] = 0x01;  // 1 Byte
			//CAN_Data[6] = 0x01;
			PORTA.OUTCLR = PIN1_bm;
			_delay_us(0.08);
			SPIC_Write(write_command);
			SPIC_Write(TXB2CTRL);
			SPIC_Write(0x03); // High Prio message
			SPIC_write_tx_buffer(p_tx_can1);
			//SPIC_Write_array(CAN_Data,12);
			PORTA.OUTSET = PIN1_bm;
			_delay_us(10);
			PORTA.OUTCLR = PIN1_bm;
			_delay_us(0.08);
			SPIC_Write(send_buffer_1);
			PORTA.OUTSET = PIN1_bm;
			
		}
		
		if((info->tm_sec > 45)  && (CAN_send))
		{
			CAN_send = false;
		}
		
		
		if(info->tm_sec >= 40 && neue_Stunde)
		neue_Stunde = false;
		
		if(force_summertime_check)
		{
			force_summertime_check = false;
			summertime(info);
		}
		
		if(ESP_complete)
		{
			ESP_complete = false;
			respond = ESP_buffer[0];
			send_answer = true;
			
		}
		
		//Save firmware to flash
		if(firmware_ready)
		{
			
			
				if(first_byte)
				{
					if(!ready_to_write)
					{
						PORTA.OUTCLR = PIN1_bm;
						_delay_us(0.08);
						SPIC_Write(Flash_RDSR);
						if(!(SPIC_Read_Write(0xFF) & 0x01) )
							ready_to_write = true;
						PORTA.OUTSET = PIN1_bm;
						
						
					}
					if(!cleared)
					{
						Flash_set_WREN();
						_delay_us(10);
						Flash_block_erase(0);
						cleared = true;
						ready_to_write = false;
						
					}
					if(cleared && ready_to_write)
					{
						Flash_set_WREN();
						_delay_us(10);
						Flash_start_write_AAI(0,firmware[counter]);
						counter++;
						first_byte = false;
					}
				}
				else
				{
					if(!last_byte)
					{
						Flash_start_write_AAI(0,firmware[counter]);
						Flash_set_WRDI();
						firmware_ready = false;
						counter = 0;
						memset_volatile(firmware,0xFF,256);
					}
					else
					{
						Flash_start_write_AAI(0,firmware[counter]);
						counter++;
						if(counter >= 256)
						{
							Flash_set_WRDI();
							firmware_ready = false;
							counter = 0;
							memset_volatile(firmware,0xFF,256);
							p_tx_can1->ID = Firmware;
							p_tx_can1->Remote_frame = true;
							p_tx_can1->length = 0;
							PORTA.OUTCLR = PIN1_bm;
							_delay_us(0.08);
							SPIC_Write(write_command);
							SPIC_Write(TXB1CTRL);
							SPIC_Write(0x00);
							SPIC_write_tx_buffer(p_tx_can1);
							PORTA.OUTSET = PIN1_bm;
							_delay_us(10);
							PORTA.OUTCLR = PIN1_bm;
							_delay_us(0.08);
							SPIC_Write(send_buffer_1);
							PORTA.OUTSET = PIN1_bm;  // Request new Firmwareframe
						}
						
					}
				}
		}
		
		
		if(send_answer)
		{
			send_answer = false;
			switch (respond)
			{
				case 's':
				{
					set_time = true;
					Unixtimestamp = 0;
					for(uint8_t i = 0;i<sizeof(ESP_buffer);i++)
					{
						if((ESP_buffer[i] >= '0') && (ESP_buffer[i] <='9'))
						{
							Unixtimestamp = Unixtimestamp * 10;
							Unixtimestamp += ESP_buffer[i] -'0';
						}
					}
					info = gurke(&Unixtimestamp,CET);
					force_summertime_check=true;
					set_time = false;
				}
				break;
				case 'H':
				{
					
				}
				case 'd':
				{
					if(info->tm_hour>9) //Stunde
					{
						itoa(info->tm_hour,Buffer,10);
						strcpy(send_buffer,Buffer);
						strcat(send_buffer,":");
					}
					
					else
					{
						strcpy(send_buffer,"0");
						itoa(info->tm_hour,Buffer,10);
						strcat(send_buffer,Buffer);
						strcat(send_buffer,":");
						
					}
					
					if(info->tm_min > 9) //Minute
					{
						itoa(info->tm_min,Buffer,10);
						strcat(send_buffer,Buffer);
						strcat(send_buffer,":");
					}
					
					else
					{
						strcat(send_buffer,"0");
						itoa(info->tm_min,Buffer,10);
						strcat(send_buffer,Buffer);
						strcat(send_buffer,":");
						
					}
					
					if(info->tm_sec > 9) //Sekunde
					{
						itoa(info->tm_sec,Buffer,10);
						strcat(send_buffer,Buffer);

					}
					
					else
					{
						strcat(send_buffer,"0");
						itoa(info->tm_sec,Buffer,10);
						strcat(send_buffer,Buffer);
						
						
					}
					strcat(send_buffer,"\r\n"); //12 Zeigen
					if(info->tm_mday > 9) //Tag
					{
						itoa(info->tm_mday,Buffer,10);
						strcat(send_buffer,Buffer);
						strcat(send_buffer,".");
					}
					
					else
					{
						strcat(send_buffer,"0");
						itoa(info->tm_mday,Buffer,10);
						strcat(send_buffer,Buffer);
						strcat(send_buffer,".");
						
					}
					if(info->tm_mon > 9) //Tag
					{
						itoa(info->tm_mon,Buffer,10);
						strcat(send_buffer,Buffer);
						strcat(send_buffer,".");
					}
					
					else
					{
						strcat(send_buffer,"0");
						itoa(info->tm_mon,Buffer,10);
						strcat(send_buffer,Buffer);
						strcat(send_buffer,".");
						
					}

					itoa(info->tm_year+1900,Buffer,10);
					strcat(send_buffer,Buffer);
					strcat(send_buffer,"\r\n");
					char helper[] = "Aktueller Unixtimestamp: ";
					strcat(send_buffer,helper);
					ultoa(Unixtimestamp,int_buffer,10);
					strcat(send_buffer,int_buffer);
					if(info->tm_isdst)
					{
						strcat(send_buffer,"Sommerzeit");
					}
					else
					{
						strcat(send_buffer,"Winterzeit");
					}
					send_buffer[strlen(send_buffer)] = 0x90; //String ENDE
					DMA_TX_ESP_CHANNEL.ADDRL = ((uint16_t)tx_buffer_Pointer >> 0) & 0xff;
					DMA_TX_ESP_CHANNEL.ADDRH = ((uint16_t)tx_buffer_Pointer >> 8) & 0xff;
					DMA_TX_ESP_CHANNEL.TRFCNT = strlen (send_buffer);
					DMA_TX_ESP_CHANNEL.CTRLA |= EDMA_CH_ENABLE_bm;
					
				}
				break;
				case 'A':
				{
					
				}
				break;
				case 'z':
				{
					char helper[] = "Status: ";
					strcpy(send_buffer,helper);
					PORTC.OUTCLR = PIN2_bm;
					SPIC_Write(Flash_RDSR);
					uint8_t data = 0;
					data = SPIC_Read_Write(0x55);
					utoa(data,int_buffer,2);
					strcat(send_buffer,int_buffer);
					Send_UART_D0(send_buffer);
					memset(send_buffer,'\0', sizeof(send_buffer));
				}
				break;
				case 'f':   //Read Flash
				{
					uint16_t adresse = 0;
					char helper[] = ":10";
					strcpy(send_buffer,helper);
					utoa(adresse,int_buffer,16);
					if(adresse<0x10)
					{
						strcat(send_buffer,"000");
						strcat(send_buffer,int_buffer);
					}
					if(adresse>=0x10 && adresse<0x100)
					{
						strcat(send_buffer,"00");
						strcat(send_buffer,int_buffer);
					}
					if(adresse>=0x100 && adresse<0x1000)
					{
						strcat(send_buffer,"0");
						strcat(send_buffer,int_buffer);
					}
					if(adresse>=0x1000)
					{
						strcat(send_buffer,int_buffer);
					}
					strcat(send_buffer,"00");  // Data Record (Typ 00)
					PORTC.OUTCLR = PIN2_bm;
					SPIC_Write(Flash_Read);
					SPIC_Write(0);
					SPIC_Write(0);
					SPIC_Write(0);
					uint8_t data = 0;
					uint8_t checksum = 0;
					for(uint8_t i=0;i<16;i++)
					{
						data = SPIC_Read_Write(0x55);
						utoa(data,int_buffer,16);
						strcat(send_buffer,int_buffer);
						adresse++;
						checksum += data;
					}
					
					checksum ^= 0xFF;
					checksum++;
					utoa(checksum,int_buffer,16);
					strcat(send_buffer,int_buffer); //Send Data
					Send_UART_D0(send_buffer);
					memset(send_buffer,'\0', sizeof(send_buffer));
					
					for(uint16_t j = 1;j<2048; j++)
					{
						checksum = 0;
						strcpy(send_buffer,helper);
						utoa(adresse,int_buffer,16);
						if(adresse<0x10)
						{
							strcat(send_buffer,"000");
							strcat(send_buffer,int_buffer);
						}
						if(adresse>=0x10 && adresse<0x100)
						{
							strcat(send_buffer,"00");
							strcat(send_buffer,int_buffer);
						}
						if(adresse>=0x100 && adresse<0x1000)
						{
							strcat(send_buffer,"0");
							strcat(send_buffer,int_buffer);
						}
						if(adresse>=0x1000)
						{
							strcat(send_buffer,int_buffer);
						}
						strcat(send_buffer,"00");  // Data Record (Typ 00)
						for(uint8_t i=0;i<16;i++)
						{
							data = SPIC_Read_Write(0x55);
							utoa(data,int_buffer,16);
							strcat(send_buffer,int_buffer);
							adresse++;
							checksum += data;
						}
						
						checksum ^= 0xFF;
						checksum++;
						utoa(checksum,int_buffer,16);
						strcat(send_buffer,int_buffer); //Send Data
						Send_UART_D0(send_buffer);
						memset(send_buffer,'\0', sizeof(send_buffer));
						
					}
					
					PORTC.OUTSET = PIN2_bm;
					
					
					
				}
				break;
				case 't':
				{
					char helper[] = "Aktueller Unixtimestamp: ";
					strcat(send_buffer,helper);
					ultoa(Unixtimestamp,int_buffer,10);
					strcat(send_buffer,int_buffer);
					send_buffer[strlen(send_buffer)] = 0x90; //String ENDE
					DMA_TX_ESP_CHANNEL.ADDRL = ((uint16_t)tx_buffer_Pointer >> 0) & 0xff;
					DMA_TX_ESP_CHANNEL.ADDRH = ((uint16_t)tx_buffer_Pointer >> 8) & 0xff;
					DMA_TX_ESP_CHANNEL.TRFCNT = strlen (send_buffer);
					DMA_TX_ESP_CHANNEL.CTRLA |= EDMA_CH_ENABLE_bm;
				}
				break;
				case 'C':
				{
					can_control = true;
					char helper[] = "CAN Control aktivier\r\n";
					strcat(send_buffer,helper);
					send_buffer[strlen(send_buffer)] = 0x90; //String ENDE
					DMA_TX_ESP_CHANNEL.ADDRL = ((uint16_t)tx_buffer_Pointer >> 0) & 0xff;
					DMA_TX_ESP_CHANNEL.ADDRH = ((uint16_t)tx_buffer_Pointer >> 8) & 0xff;
					DMA_TX_ESP_CHANNEL.TRFCNT = strlen (send_buffer);
					DMA_TX_ESP_CHANNEL.CTRLA |= EDMA_CH_ENABLE_bm;
				}
				break;
				case 'c':
				{
					can_control = false;
					char helper[] = "CAN Control deaktivier\r\n";
					strcat(send_buffer,helper);
					send_buffer[strlen(send_buffer)] = 0x90; //String ENDE
					DMA_TX_ESP_CHANNEL.ADDRL = ((uint16_t)tx_buffer_Pointer >> 0) & 0xff;
					DMA_TX_ESP_CHANNEL.ADDRH = ((uint16_t)tx_buffer_Pointer >> 8) & 0xff;
					DMA_TX_ESP_CHANNEL.TRFCNT = strlen (send_buffer);
					DMA_TX_ESP_CHANNEL.CTRLA |= EDMA_CH_ENABLE_bm;
				}
				break;
				case 'W':
				{
					zaehlerstand = 0;
					for(uint8_t i = 0;i<sizeof(ESP_buffer);i++)
					{
						if((ESP_buffer[i] >= '0') && (ESP_buffer[i] <='9'))
						{
							zaehlerstand = zaehlerstand * 10;
							zaehlerstand += ESP_buffer[i] -'0';
						}
					}
					Zaehlerstand_alt = zaehlerstand;
					buffer_FRAM[0] = (uint8_t) (Zaehlerstand_alt>>24);
					buffer_FRAM[1] = (uint8_t) (Zaehlerstand_alt>>16);
					buffer_FRAM[2] = (uint8_t) (Zaehlerstand_alt>>8);
					buffer_FRAM[3] = (uint8_t) Zaehlerstand_alt;
					buffer_FRAM[4] = (uint8_t) (Verbrauch_heute>>8);
					buffer_FRAM[5] = (uint8_t) Verbrauch_heute;
					twi_write(&TWIC,buffer_FRAM,FRAM_base_adress,0,6,false);
					ultoa(zaehlerstand,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,":");
					uint16_t Rest = zaehlerstand%1000;
					ultoa(zaehlerstand/1000,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,".");
					if(Rest < 10)
					strcat(send_buffer,"00");
					if((Rest >= 10) && (Rest < 100))
					strcat(send_buffer,"0");
					ultoa(Rest,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,":");
					ultoa(Verbrauch_heute/1000,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,".");
					Rest = Verbrauch_heute%1000;
					if(Rest < 10)
					strcat(send_buffer,"00");
					if((Rest >= 10) && (Rest < 100))
					strcat(send_buffer,"0");
					ultoa(Rest,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,":");
					ultoa(Verbrauch_gestern/1000,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,".");
					Rest = Verbrauch_gestern%1000;
					if(Rest < 10)
					strcat(send_buffer,"00");
					if((Rest >= 10) && (Rest < 100))
					strcat(send_buffer,"0");
					ultoa(Rest,int_buffer,10);
					strcat(send_buffer,int_buffer);
					
					send_buffer[strlen(send_buffer)] = 0x90; //String ENDE
					DMA_TX_ESP_CHANNEL.ADDRL = ((uint16_t)tx_buffer_Pointer >> 0) & 0xff;
					DMA_TX_ESP_CHANNEL.ADDRH = ((uint16_t)tx_buffer_Pointer >> 8) & 0xff;
					DMA_TX_ESP_CHANNEL.TRFCNT = strlen (send_buffer);
					DMA_TX_ESP_CHANNEL.CTRLA |= EDMA_CH_ENABLE_bm;
				}
				break;
				case 'w':
				{
					
					ultoa(zaehlerstand,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,":");
					uint16_t Rest = zaehlerstand%1000;
					ultoa(zaehlerstand/1000,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,".");
					if(Rest < 10)
					strcat(send_buffer,"00");
					if((Rest >= 10) && (Rest < 100))
					strcat(send_buffer,"0");
					ultoa(Rest,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,":");
					ultoa(Verbrauch_heute/1000,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,".");
					Rest = Verbrauch_heute%1000;
					if(Rest < 10)
					strcat(send_buffer,"00");
					if((Rest >= 10) && (Rest < 100))
					strcat(send_buffer,"0");
					ultoa(Rest,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,":");
					ultoa(Verbrauch_gestern/1000,int_buffer,10);
					strcat(send_buffer,int_buffer);
					strcat(send_buffer,".");
					Rest = Verbrauch_gestern%1000;
					if(Rest < 10)
					strcat(send_buffer,"00");
					if((Rest >= 10) && (Rest < 100))
					strcat(send_buffer,"0");
					ultoa(Rest,int_buffer,10);
					strcat(send_buffer,int_buffer);
					send_buffer[strlen(send_buffer)] = 0x90; //String ENDE
					DMA_TX_ESP_CHANNEL.ADDRL = ((uint16_t)tx_buffer_Pointer >> 0) & 0xff;
					DMA_TX_ESP_CHANNEL.ADDRH = ((uint16_t)tx_buffer_Pointer >> 8) & 0xff;
					DMA_TX_ESP_CHANNEL.TRFCNT = strlen (send_buffer);
					DMA_TX_ESP_CHANNEL.CTRLA |= EDMA_CH_ENABLE_bm;
				}
				break;
				case 'v':
				{
					
					break;
					default:
					{
						strcpy(send_buffer,"Falsche Eingabe");
						send_buffer[strlen(send_buffer)] = 0x90;  //String ENDE
						DMA_TX_ESP_CHANNEL.ADDRL = ((uint16_t)tx_buffer_Pointer >> 0) & 0xff;
						DMA_TX_ESP_CHANNEL.ADDRH = ((uint16_t)tx_buffer_Pointer >> 8) & 0xff;
						DMA_TX_ESP_CHANNEL.TRFCNT = strlen (send_buffer);
						DMA_TX_ESP_CHANNEL.CTRLA |= EDMA_CH_ENABLE_bm;
					}
					break;
				}
				
			}
			memset(ESP_buffer,'\0', sizeof(ESP_buffer));
		}
		
	}
}

