/*
 * MCP2515.h
 *
 * Created: 06.05.2017 00:13:10
 *  Author: JackFrost
 */ 


#ifndef MCP2515_H_
#define MCP2515_H_

//SPI commands

#define reset_command 0xC0
#define write_command 0x02
#define read_command 0x03
#define send_buffer_0 0x81
#define send_buffer_1 0x82
#define send_buffer_2 0x84
#define read_status 0xA0
#define RX_status 0xB0
#define bit_modify_command 0x05
#define Read_RX0_SID  0x90  /* Lowest priority */
#define Read_RX0_DATA  0x92  /* Filter off */
#define Read_RX1_SID  0x94  /* Lowest priority */
#define Read_RX1_DATA  0x96  /* Filter off */

//Bits
//Transmit buffers
#define ABTF_bp		6		//Message Aborted Flag bit position
#define ABTF_gm		0x40	//Message Aborted Flag bit mask
#define MLOA_bp		5		//Message Lost Arbitration bit position
#define MLOA_bm		0x20	//Message Lost Arbitration bit mask
#define TXERR_bp	4		//Transmission Error Detected bit position
#define TXERR_bm	0x10	//Transmission Error Detected bit mask
#define TXTREQ_bp	3		//Message Transmit Request bit position
#define TXREQ_bm	0x08	//Message Transmit Request bit mask
#define TXP1_bp		1		//Transmit Buffer Priority bits position
#define	TXP1_bm		0x02	//Transmit Buffer Priority bits mask
#define TXP0_bp		0		//Transmit Buffer Priority bits position
#define TXP0_bm		0x01	//Transmit Buffer Priority bits mask

typedef enum Transmit_Buffer_Priority
{
	Transmit_Buffer_Priority_LOWEST_gc = (0x00<<0),  /* Lowest priority */
	Transmit_Buffer_Priority_LO_gc = (0x01<<0),  /* Low priority */
	Transmit_Buffer_Priority_MED_gc = (0x02<<0),  /* Medium priority */
	Transmit_Buffer_Priority_HIGH_gc = (0x03<<0),  /* High priority */
} Transmit_Buffer_Priority_t;

//PIN CONTROL AND STATUS REGISTER
#define B2RTS_bp	5		// TX2RTS Pin State bit position
#define B2RTS_bm	0x20	// TX2RTS Pin State bit mask
#define B1RTS_bp	4		// TX1RTS Pin State bit position
#define B1RTS_bm	0x10	// TX1RTS Pin State bit mask
#define B0RTS_bp	3		// TX0RTS Pin State bit position
#define B0RTS_bm	0x08	// TX0RTS Pin State bit mask
#define B2RTSM_bp	2		// TX2RTS Pin mode bit position
#define B2RTSM_bm	0x04	// TX2RTS Pin mode bit mask
#define B1RTSM_bp	1		// TX1RTS Pin mode bit position
#define B1RTSM_bm	0x02	// TX1RTS Pin mode bit mask	
#define B0RTSM_bp	0		// TX0RTS Pin mode bit position
#define B0RTSM_bm	0x01	// TX0RTS Pin mode bit mask

#define EXIDE_bp	3		//Extended Identifier Enable bit position
#define EXIDE_bm	0x08	//Extended Identifier Enable bit mask

#define RTR_bp		6		//Remote Transmission Request bit position
#define RTR_bm		0x40	//Remote Transmission Request bit mask

//Recieve bits
#define RXM1_bp		6		//Receive Buffer Operating mode bits position
#define RXM1_bm		0x40	//Receive Buffer Operating mode bits mask
#define RXM0_bp		5		//Receive Buffer Operating mode bits position
#define RXM0_bm		0x20	//Receive Buffer Operating mode bits mask

typedef enum Receive_Buffer_Operating_mode
{
	Receive_Buffer_filter_on_gc = (0x00<<5),  /* Lowest priority */
	Receive_Buffer_filter_off_gc = (0x03<<5),  /* Filter off */
} Receive_Buffer_Operating_mode_t;

#define RXRTR_bp	3		//Received Remote Transfer Request bit position
#define RXRTR_bm	0x08	//Received Remote Transfer Request bit mask
#define BUKT_bp		2		//Rollover Enable bit position
#define BUKT_bm		0x04	//Rollover Enable bit mask
#define FILHIT2_bp	2		//Filter Hit bits
#define FILHIT2_bm  0x04	//Filter Hit bits
#define FILHIT1_bp	1		//Filter Hit bits
#define FILHIT1_bm  0x02	//Filter Hit bits
#define FILHIT0_bp	0		//Filter Hit bits
#define FILHIT0_bm  0x01	//Filter Hit bits

#define B1BFS_bp	5		// RX1BF Pin State bit position
#define B1BFS_bm	0x20	// RX1BF Pin State bit mask
#define B0BFS_bp	4		// RX1BF Pin State bit position
#define B0BFS_bm	0x10	// RX1BF Pin State bit mask
#define B1BFE_bp	3		// RX1BF Pin Function Enable bit position
#define B1BFE_bm	0x08	// RX1BF Pin Function Enable bit mask
#define B0BFE_bp	2		// RX1BF Pin Function Enable bit position
#define B0BFE_bm	0x04	// RX1BF Pin Function Enable bit mask
#define B1BFM_bp	1		// RX1BF Pin Operation mode bit position
#define B1BFM_bm	0x02	// RX1BF Pin Operation mode bit mask
#define B0BFM_bp	0		// RX1BF Pin Operation mode bit position
#define B0BFM_bm	0x01	// RX1BF Pin Operation mode bit mask

#define SRR_bp		4		//Standard Frame Remote Transmit Request bit position
#define SRR_bm		0x10	//Standard Frame Remote Transmit Request bit mask
#define IDE_bp		3		//Extended Identifier Flag bit position
#define IDE_bm		0x08	//Extended Identifier Flag bit mask
#define RTR_bp		6		//Extended Frame Remote Transmission Request bit position
#define RTR_bm		0x40	//Extended Frame Remote Transmission Request bit mask

#define EXIDE_bp	3		//Extended Identifier Enable bit position
#define EXIDE_bm	0x08	//Extended Identifier Enable bit mask

//Configuration bits
#define SJW1_bp		7		//Synchronization Jump Width Length bit position
#define SJW1_bm		0x80	//Synchronization Jump Width Length bit mask
#define SJW0_bp		6		//Synchronization Jump Width Length bit position
#define SJW0_bm		0x40	//Synchronization Jump Width Length bit mask
#define SJW_bp		6		//Synchronization Jump Width Length bits position

typedef enum Synchronization_Jump_Width_Length
{
	one_time_quanta_gc = (0x00<<6),  /* 1x Tq Sync */
	two_time_quanta_gc = (0x01<<6),  /* 2x Tq Syn */
	three_time_quanta_gc = (0x02<<6),  /* 3x Tq Syn */
	four_time_quanta_gc = (0x03<<6),  /* 4x Tq Syn */
} Synchronization_Jump_Width_Length_t;

#define BTLMODE_bp	7		//PS2 Bit Time Length bit position
#define BTLMODE_bm	0x80	//PS2 Bit Time Length bit mask
#define SAM_bp		6		//Sample Point Configuration bit
#define SAM_bm		0x40	//Sample Point Configuration bit
#define PHSEG1_bp	3		//PS1 Length bis position (PHSEG1<2:0> + 1) x TQ
#define PRSEG_bp	0		//Propagation Segment Length bits position (PRSEG<2:0> + 1) x TQ

#define SOF_bp		7		//Start-of-Frame signal bit position
#define SOF_bm		0x80	//Start-of-Frame signal bit mask
#define WAKFIL_bp	6		//Wake-up Filter bit position
#define WAKFIL_bm	0x40	//Wake-up Filter bit mask
#define PHSEG2_bp	0		//PS2 Length bits position

//Error
#define RX1OVR_bp	7		//Receive Buffer 1 Overflow Flag bit position
#define RX1OVR_bm	0x80	//Receive Buffer 1 Overflow Flag bit mask
#define RX0OVR_bp	6		//Receive Buffer 0 Overflow Flag bit position
#define RX0OVR_bm	0x40	//Receive Buffer 0 Overflow Flag bit mask
#define TXBO_bp		5		//Bus-Off Error Flag bit position
#define TXBO_bm		0x20	//Bus-Off Error Flag bit mask
#define TXEP_bp		4		//Transmit Error-Passive Flag bit position
#define TXEP_bm		0x10	//Transmit Error-Passive Flag bit mask
#define RXEP_bp		3		//Receive Error-Passive Flag bit position
#define RXEP_bm		0x08	//Receive Error-Passive Flag bit mask
#define TXWAR_bp	2		//Transmit Error Warning Flag bit position
#define TXWAR_bm	0x04	//Transmit Error Warning Flag bit mask
#define RXWAR_bp	1		//Receive Error Warning Flag bit position
#define RXWAR_bm	0x02	//Receive Error Warning Flag bit mask
#define EWARN_bp	0		// Error Warning Flag bit position
#define EWARN_bm	0x01	// Error Warning Flag bit mask

//Interrupt
#define MERRE_bp	7		//Message Error Interrupt Enable bit position
#define MERRE_bm	0x80	//Message Error Interrupt Enable bit mask
#define WAKIE_bp	6		//Wake-up Interrupt Enable bit position
#define WAKIE_bm	0x40	//Wake-up Interrupt Enable bit mask
#define ERRIE_bp	5		//Error Interrupt Enable bit position
#define ERRIE_bm	0x20	//Error Interrupt Enable bit mask
#define TX2IE_bp	4		//Transmit Buffer 2 Empty Interrupt Enable bit position
#define TX2IE_bm	0x10	//Transmit Buffer 2 Empty Interrupt Enable bit mask
#define TX1IE_bp	3		//Transmit Buffer 1 Empty Interrupt Enable bit position
#define TX1IE_bm	0x08	//Transmit Buffer 1 Empty Interrupt Enable bit mask
#define TX0IE_bp	2		//Transmit Buffer 0 Empty Interrupt Enable bit position
#define TX0IE_bm	0x04	//Transmit Buffer 0 Empty Interrupt Enable bit mask
#define RX1IE_bp	1		//Receive Buffer 1 Full Interrupt Enable bit position
#define RX1IE_bm	0x02	//Receive Buffer 1 Full Interrupt Enable bit mask
#define RX0IE_bp	0		//Receive Buffer 0 Full Interrupt Enable bit position
#define RX0IE_bm	0x01	//Receive Buffer 0 Full Interrupt Enable bit mask

#define MERRF_bp	7		//Message Error Interrupt Flag bit position
#define MERRF_bm	0x80	//Message Error Interrupt Flag bit mask
#define WAKIF_bp	6		//Wake-up Interrupt Flag bit position
#define WAKIF_bm	0x40	//Wake-up Interrupt Flag bit mask
#define ERRIF_bp	5		//Error Interrupt Flag bit position
#define ERRIF_bm	0x20	//Error Interrupt Flag bit mask
#define TX2IF_bp	4		//Transmit Buffer 2 Empty Interrupt Flag bit position
#define TX2IF_bm	0x10	//Transmit Buffer 2 Empty Interrupt Flag bit mask
#define TX1IF_bp	3		//Transmit Buffer 1 Empty Interrupt Flag bit position
#define TX1IF_bm	0x08	//Transmit Buffer 1 Empty Interrupt Flag bit mask
#define TX0IF_bp	2		//Transmit Buffer 0 Empty Interrupt Flag bit position
#define TX0IF_bm	0x04	//Transmit Buffer 0 Empty Interrupt Flag bit mask
#define RX1IF_bp	1		//Receive Buffer 1 Full Interrupt Flag bit position
#define RX1IF_bm	0x02	//Receive Buffer 1 Full Interrupt Flag bit mask
#define RX0IF_bp	0		//Receive Buffer 0 Full Interrupt Flag bit position
#define RX0IF_bm	0x01	//Receive Buffer 0 Full Interrupt Flag bit mask

//Configuration
#define REQOP2_bp	7		//Request Operation Mode bit position
#define REQOP2_bm	0x80	//Request Operation Mode bit mask
#define REQOP1_bp	6		//Request Operation Mode bit position
#define REQOP1_bm	0x40	//Request Operation Mode bit mask
#define REQOP0_bp	5		//Request Operation Mode bit position
#define REQOP0_bm	0x20	//Request Operation Mode bit mask
#define ABAT_bp		4		//Abort All Pending Transmissions bit position
#define ABAT_bm		0x10	//Abort All Pending Transmissions bit mask
#define OSM_bp		3		//One-Shot Mode bit position
#define OSM_bm		0x08	//One-Shot Mode bit mask
#define CLKEN_bp	2		//CLKOUT Pin Enable bit position
#define CLKEN_bm	0x04	//CLKOUT Pin Enable bit mask
#define CLKPRE1_bp	1		//CLKOUT Pin Prescaler bit position
#define CLKPRE1_bm	0x02	//CLKOUT Pin Prescaler bit mask
#define CLKPRE0_bp	0		//CLKOUT Pin Prescaler bit position
#define CLKPRE0_bm	0x01	//CLKOUT Pin Prescaler bit mask

typedef enum Request_Operation_Mode
{
	normal_operation_mode_gc = (0x00<<5),	/* Sets Normal Operation mode */
	sleep_mode_gc = (0x01<<5),				/* Sets Sleep mode */
	loopback_mode_gc = (0x02<<5),			/* Sets Loopback mode */
	listen_only_mode_gc = (0x03<<5),		/* Sets Listen-Only mode */
	configuration_mode_gc = (0x04<<5),		/* Sets Configuration mode */
} Request_Operation_Mode_t;

typedef enum CLKOUT_Pin_Prescaler
{
	system_clock_gc = (0x00<<0),			/* System Clock */
	half_system_clock_gc = (0x01<<0),		/* System Clock/2 */
	quarter_system_clock_gc = (0x02<<0),	/* System Clock/4 */
	eighth_system_clock_gc = (0x03<<0),		/* System Clock/8 */
} CLKOUT_Pin_Prescaler_t;

#define OPMOD_bm	0xE0	//Operation Mode bits mask
#define ICOD_bm		0x0E	//Interrupt Flag Code bits  mask

typedef enum Interrupt_Flag_Code
{
	no_interrupt_gc = (0x00<<1),			/* No interrupt */
	error_interrupt_gc = (0x01<<1),			/* Error interrupt */
	wake_up_interrupt_gc = (0x02<<1),		/* Wake-up interrupt */
	TXB0_interrupt_gc = (0x03<<1),			/* TXB0 interrupt */
	TXB1_interrupt_gc = (0x04<<1),			/* TXB1 interrupt */
	TXB2_interrupt_gc = (0x05<<1),			/* TXB2 interrupt */
	RXB0_interrupt_gc = (0x06<<1),			/* RXB0 interrupt */
	RXB1_interrupt_gc = (0x07<<1),			/* RXB1 interrupt */
} Interrupt_Flag_Code;

// Registers
//Transmit buffers
 
#define TXBRTSCTRL	0x0D	//PIN CONTROL AND STATUS REGISTER

//TXB Buffer0
#define TXB0CTRL	0x30	//CONTROL REGISTER
#define TXB0SIDH	0x31	//STANDARD IDENTIFIER REGISTER HIGH
#define TXB0SIDL	0x32	//STANDARD IDENTIFIER REGISTER LOW
#define TXB0EID8	0x33	//EXTENDED IDENTIFIER 8 REGISTER HIGH
#define TXB0EID0	0x34	//EXTENDED IDENTIFIER 0 REGISTER LOW
#define TXB0DLC		0x35	//DATA LENGTH CODE REGISTER
#define TXB0D0		0x36	//DATA BYTE 0 REGISTER
#define TXB0D1		0x37	//DATA BYTE 1 REGISTER
#define TXB0D2		0x38	//DATA BYTE 2 REGISTER
#define TXB0D3		0x39	//DATA BYTE 3 REGISTER
#define TXB0D4		0x3A	//DATA BYTE 4 REGISTER
#define TXB0D5		0x3B	//DATA BYTE 5 REGISTER
#define TXB0D6		0x3C	//DATA BYTE 6 REGISTER
#define TXB0D7		0x3D	//DATA BYTE 7 REGISTER

//TXB Buffer1
#define TXB1CTRL	0x40 //CONTROL REGISTER
#define TXB1SIDH	0x41	//STANDARD IDENTIFIER REGISTER HIGH
#define TXB1SIDL	0x42	//STANDARD IDENTIFIER REGISTER LOW 
#define TXB1EID8 	0x43	//EXTENDED IDENTIFIER 8 REGISTER HIGH
#define TXB1EID0	0x44	//EXTENDED IDENTIFIER 0 REGISTER LOW 
#define TXB1DLC		0x45	//DATA LENGTH CODE REGISTER		 
#define TXB1D0		0x46	//DATA BYTE 0 REGISTER 
#define TXB1D1 		0x47	//DATA BYTE 1 REGISTER
#define TXB1D2		0x48	//DATA BYTE 2 REGISTER 
#define TXB1D3		0x49	//DATA BYTE 3 REGISTER
#define TXB1D4		0x4A	//DATA BYTE 4 REGISTER
#define TXB1D5		0x4B	//DATA BYTE 5 REGISTER
#define TXB1D6		0x4C	//DATA BYTE 6 REGISTER
#define TXB1D7		0x4D	//DATA BYTE 7 REGISTER

//TXB Buffer2
#define TXB2CTRL	0x50	//CONTROL REGISTER
#define TXB2SIDH	0x51	//STANDARD IDENTIFIER REGISTER HIGH
#define TXB2SIDL	0x52	//STANDARD IDENTIFIER REGISTER LOW
#define TXB2EID8	0x53	//EXTENDED IDENTIFIER 8 REGISTER HIGH
#define TXB2EID0	0x54	//EXTENDED IDENTIFIER 0 REGISTER LOW
#define TXB2DLC		0x55	//DATA LENGTH CODE REGISTER
#define TXB2D0		0x56	//DATA BYTE 0 REGISTER
#define TXB2D1		0x57	//DATA BYTE 1 REGISTER
#define TXB2D2		0x58	//DATA BYTE 2 REGISTER
#define TXB2D3		0x59	//DATA BYTE 3 REGISTER
#define TXB2D4		0x5A	//DATA BYTE 4 REGISTER
#define TXB2D5		0x5B	//DATA BYTE 5 REGISTER
#define TXB2D6		0x5C	//DATA BYTE 6 REGISTER
#define TXB2D7		0x5D	//DATA BYTE 7 REGISTER

//Recieve buffer

#define BFPCTRL		0x0C	//PIN CONTROL AND STATUS REGISTER

//RX0 Buffer
#define RXB0CTRL	0x60	//CONTROL REGISTER
#define RXB0SIDH	0x61	//STANDARD IDENTIFIER REGISTER HIGH
#define RXB0SIDL	0x62	//STANDARD IDENTIFIER REGISTER LOW
#define RXB0EID8	0x63	//EXTENDED IDENTIFIER REGISTER HIGH
#define RXB0EID0	0x64	//EXTENDED IDENTIFIER REGISTER LOW
#define RXB0DLC		0x65	//DATA LENGTH CODE REGISTER
#define RXB0D0		0x66	//DATA BYTE 0 REGISTER
#define RXB0D1		0x67	//DATA BYTE 1 REGISTER
#define RXB0D2		0x68	//DATA BYTE 2 REGISTER
#define RXB0D3		0x69	//DATA BYTE 3 REGISTER
#define RXB0D4		0x6A	//DATA BYTE 4 REGISTER
#define RXB0D5		0x6B	//DATA BYTE 5 REGISTER
#define RXB0D6		0x6C	//DATA BYTE 6 REGISTER
#define RXB0D7		0x6D	//DATA BYTE 7 REGISTER

//RX1 Buffer
#define RXB1CTRL	0x70	//CONTROL REGISTER
#define RXB1SIDH	0x71	//STANDARD IDENTIFIER REGISTER HIGH
#define RXB1SIDL	0x72	//STANDARD IDENTIFIER REGISTER LOW
#define RXB1EID8	0x73	//EXTENDED IDENTIFIER REGISTER HIGH
#define RXB1EID0	0x74	//EXTENDED IDENTIFIER REGISTER LOW
#define RXB1DLC		0x75	//DATA LENGTH CODE REGISTER
#define RXB1D0		0x76	//DATA BYTE 0 REGISTER
#define RXB1D1		0x77	//DATA BYTE 1 REGISTER
#define RXB1D2		0x78	//DATA BYTE 2 REGISTER
#define RXB1D3		0x79	//DATA BYTE 3 REGISTER
#define RXB1D4		0x7A	//DATA BYTE 4 REGISTER
#define RXB1D5		0x7B	//DATA BYTE 5 REGISTER
#define RXB1D6		0x7C	//DATA BYTE 6 REGISTER
#define RXB1D7		0x7D	//DATA BYTE 7 REGISTER

//Recieve Filters
//Filter 0
#define RXF0SIDH	0x00	//STANDARD IDENTIFIER REGISTER HIGH
#define RXF0SIDL	0x01	//STANDARD IDENTIFIER REGISTER LOW
#define RXF0EID8	0x02	//EXTENDED IDENTIFIER REGISTER HIGH
#define RXF0EID0	0x03	//EXTENDED IDENTIFIER REGISTER LOW

//Filter 1
#define RXF1SIDH	0x04	//STANDARD IDENTIFIER REGISTER HIGH
#define RXF1SIDL	0x05	//STANDARD IDENTIFIER REGISTER LOW
#define RXF1EID8	0x06	//EXTENDED IDENTIFIER REGISTER HIGH
#define RXF1EID0	0x07	//EXTENDED IDENTIFIER REGISTER LOW

//Filter 2
#define RXF2SIDH	0x08	//STANDARD IDENTIFIER REGISTER HIGH
#define RXF2SIDL	0x09	//STANDARD IDENTIFIER REGISTER LOW
#define RXF2EID8	0x0A	//EXTENDED IDENTIFIER REGISTER HIGH
#define RXF2EID0	0x0B	//EXTENDED IDENTIFIER REGISTER LOW

//Filter 3
#define RXF3SIDH	0x10	//STANDARD IDENTIFIER REGISTER HIGH
#define RXF3SIDL	0x11	//STANDARD IDENTIFIER REGISTER LOW
#define RXF3EID8	0x12	//EXTENDED IDENTIFIER REGISTER HIGH
#define RXF3EID0	0x13	//EXTENDED IDENTIFIER REGISTER LOW

//Filter 4
#define RXF4SIDH	0x14	//STANDARD IDENTIFIER REGISTER HIGH
#define RXF4SIDL	0x15	//STANDARD IDENTIFIER REGISTER LOW
#define RXF4EID8	0x16	//EXTENDED IDENTIFIER REGISTER HIGH
#define RXF4EID0	0x17	//EXTENDED IDENTIFIER REGISTER LOW

//Filter 5
#define RXF5SIDH	0x18	//STANDARD IDENTIFIER REGISTER HIGH
#define RXF5SIDL	0x19	//STANDARD IDENTIFIER REGISTER LOW
#define RXF5EID8	0x1A	//EXTENDED IDENTIFIER REGISTER HIGH
#define RXF5EID0	0x1B	//EXTENDED IDENTIFIER REGISTER LOW

//Recieve Masks
//Mask 0
#define RXM0SIDH	0x20	//STANDARD IDENTIFIER REGISTER HIGH
#define RXM0SIDL	0x21	//STANDARD IDENTIFIER REGISTER LOW
#define RXM0EID8	0x22	//EXTENDED IDENTIFIER REGISTER HIGH
#define RXM0EID0	0x23	//EXTENDED IDENTIFIER REGISTER LOW

//Mask 1
#define RXM1SIDH	0x24	//STANDARD IDENTIFIER REGISTER HIGH
#define RXM1SIDL	0x25	//STANDARD IDENTIFIER REGISTER LOW
#define RXM1EID8	0x26	//EXTENDED IDENTIFIER REGISTER HIGH
#define RXM1EID0	0x27	//EXTENDED IDENTIFIER REGISTER LOW

//Bit Timing
#define CNF1		0x2A	//CONFIGURATION REGISTER 1
#define CNF2		0x29	//CONFIGURATION REGISTER 2
#define CNF3		0x28	//CONFIGURATION REGISTER 3

//Error
#define TEC			0x1C	//TRANSMIT ERROR COUNTER REGISTER
#define REC			0x1D	//RECEIVE ERROR COUNTER REGISTER
#define	EFLG		0x2D	//ERROR FLAG REGISTER

//Interrupts
#define CANINTE		0x2B	//CAN INTERRUPT ENABLE REGISTER
#define CANINTF		0x2C	//CAN INTERRUPT FLAG REGISTER

//CAN Control
#define CANCTRL		0x0F	//CAN CONTROL REGISTER
#define CANSTAT		0x0E	//CAN STATUS REGISTER




#endif /* MCP2515_H_ */