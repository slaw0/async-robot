/********************************************************************************************************/
/*                                                                                                      */
/* Hardware Abstraction Layer                                                          */
/*                                                                                                      */
/********************************************************************************************************/
/*																										*/
/********************************************************************************************************/
/* Compiler: AVR-GCC																					*/
/* Target platform: AVRstamp1.3FUSB-CAN USPS Transmitter Configuration									*/
/********************************************************************************************************/
/********************************************************************************************************/

#ifndef HAL_H
#define HAL_H

/********************************************************************************************************/
/**************************                   AVR I/O PORTS                   ***************************/
/********************************************************************************************************/

/*****	Port A	*****************************************************************************************/

//#define LCD_RS			1 // PC.1 - Output: Alphanumeric LCD Instruction/Data Reg. Select
//#define LCD_RW			2 // PC.2 - Output: Alphanumeric LCD Read/Write
//#define LCD_EN			3 // PC.3 - Output: Alphanumeric LCD Enable
//#define LCD_D0			4 // PC.4 - Output: Alphanumeric LCD Data 0
//#define LCD_D1			5 // PC.5 - Output: Alphanumeric LCD Data 1
//#define LCD_D2			6 // PC.6 - Output: Alphanumeric LCD Data 2
//#define LCD_D3			7 // PC.7 - Output: Alphanumeric LCD Data 3

//#define LCD_DATA_FIELD	0xF0
//#define LCD_PORT			PORTA
//#define LCD_DDR			DDRA
//#define LCD_PIN			PINA

/*****	Port B	*****************************************************************************************/

#define SCK					1 // PB.1 - Output: SPI Serial Clock (SCLK)
#define MOSI				2 // PB.2 - Output: SPI Master out - slave in (MOSI)
#define MISO				3 // PB.3 - Input:  SPI Master in - slave out (MISO)
#define FLASH_CS			4 // PB.4 - Output: Flash Chip Select

/*****	Port D	*****************************************************************************************/

#define ASTAMP_LED2			0 // PD.0 - Output: AVRstamp LED2
#define ASTAMP_LED3			1 // PD.1 - Output: AVRstamp LED3
#define ASTAMP_LED4			2 // PD.2 - Output: AVRstamp LED4
#define CAN_TX				5 // PD.5 - Output: CAN Transmit Data
#define CAN_RX				6 // PD.6 - Input:  CAN Receive Data
#define CAN_RS				7 // PD.7 - Output: CAN Switch Standby

/*****	Port E	*****************************************************************************************/

#define UART0_RXD			0 // PE.0 - Input:  UART0 RXD
#define UART0_TXD			1 // PE.1 - Output: UART0 TXD
#define PWM_A				3 // PE.3 - Output: PWM Output A
#define PWM_B				4 // PE.4 - Output: PWM Output B
#define PWM_C				5 // PE.5 - Output: PWM Output C

/*****	ADC inputs	*************************************************************************************/

#define ADC_INPUT_0			0 // PF.0 - ADC0
#define ADC_INPUT_1			1 // PF.1 - ADC1
#define ADC_INPUT_2			2 // PF.2 - ADC2
#define ADC_INPUT_3			3 // PF.3 - ADC3

/*****	JTAG interface	*********************************************************************************/

// PF.4 - Input : JTAG TCK serial clock
// PF.5 - Input : JTAG TMS strobe enable
// PF.6 - Output: JTAG TDO serial data output
// PF.7 - Input : JTAG TDI serial data input

/*****	Port G	*****************************************************************************************/

/*****	Port C	*****************************************************************************************/

#define ASTAMP_7SE			0 // PC.0 - Output: AVRstamp 7seg LED e
#define ASTAMP_7SG			1 // PC.1 - Output: AVRstamp 7seg LED g
#define ASTAMP_7SD			2 // PC.2 - Output: AVRstamp 7seg LED d
#define ASTAMP_7SC			3 // PC.3 - Output: AVRstamp 7seg LED c
#define ASTAMP_7SDP			4 // PC.4 - Output: AVRstamp 7seg LED dp
#define ASTAMP_7SA			5 // PC.5 - Output: AVRstamp 7seg LED a
#define ASTAMP_7SB			6 // PC.6 - Output: AVRstamp 7seg LED b
#define ASTAMP_7SF			7 // PC.7 - Output: AVRstamp 7seg LED f

// 7seg LED codes     fba.cdge

#define AS7SEG_0	0b00010010 // Decimal or hexadecimal digit 0
#define AS7SEG_1	0b10110111 // Decimal or hexadecimal digit 1
#define AS7SEG_2	0b10011000 // Decimal or hexadecimal digit 2
#define AS7SEG_3	0b10010001 // Decimal or hexadecimal digit 2
#define AS7SEG_4	0b00110101 // Decimal or hexadecimal digit 2
#define AS7SEG_5	0b01010001 // Decimal or hexadecimal digit 2
#define AS7SEG_6	0b01010000 // Decimal or hexadecimal digit 2
#define AS7SEG_7	0b10010111 // Decimal or hexadecimal digit 2
#define AS7SEG_8	0b00010000 // Decimal or hexadecimal digit 2
#define AS7SEG_9	0b00010001 // Decimal or hexadecimal digit 2
#define AS7SEG_xA	0b00010100 // Hexecimal digit or character A
#define AS7SEG_xB	0b01110000 // Hexecimal digit or character b
#define AS7SEG_xC	0b11111000 // Hexecimal digit or character c
#define AS7SEG_xD	0b10110000 // Hexecimal digit or character d
#define AS7SEG_xE	0b01011000 // Hexecimal digit or character E
#define AS7SEG_xF	0b01011100 // Hexecimal digit or character F
#define AS7SEG_A	0b00010100 // Character A
#define AS7SEG_LB	0b01110000 // Character b
#define AS7SEG_C	0b01011010 // Character C
#define AS7SEG_LC	0b11111000 // Character c
#define AS7SEG_LD	0b10110000 // Character d
#define AS7SEG_E	0b01011000 // Character E
#define AS7SEG_F	0b01011100 // Character F
#define AS7SEG_H	0b00110100 // Character H
#define AS7SEG_LH	0b10110100 // Character h
#define AS7SEG_I	0b10110111 // Character I
#define AS7SEG_L	0b01111010 // Character L
#define AS7SEG_LN	0b11110100 // Character n
#define AS7SEG_O	0b00010010 // Character O
#define AS7SEG_LO	0b11110000 // Character o
#define AS7SEG_P	0b00011100 // Character P
#define AS7SEG_LR	0b11111100 // Character r
#define AS7SEG_S	0b01010001 // Character S
#define AS7SEG_LT	0b01111100 // Character t
#define AS7SEG_U	0b00110010 // Character U
#define AS7SEG_LU	0b11110010 // Character u
#define AS7SEG_SA	0b11011111 // Segment a
#define AS7SEG_SB	0b10111111 // Segment b
#define AS7SEG_SC	0b11110111 // Segment c
#define AS7SEG_SD	0b11111011 // Segment d
#define AS7SEG_SE	0b11111110 // Segment e
#define AS7SEG_SF	0b01111111 // Segment f
#define AS7SEG_SG	0b11111101 // Segment g
#define AS7SEG_SDP	0b11101111 // Segment dp

/*******************************************************************************************************/
/*******************************************************************************************************/
/**************************                 PORT SETUP MACROS                 **************************/
/*******************************************************************************************************/
/*******************************************************************************************************/

/*******************************************************************************************************/
/**************************                PORT INITIALIZATION                **************************/
/*******************************************************************************************************/

// Disables pull-up on all inputs!!!
#define PORT_INIT() \
    do { \
        MCUCR |= BM(PUD); \
        DDRA  = 0x00; \
        PORTA = 0x00; \
        DDRB  = BM(MOSI) | BM(SCK) | BM(FLASH_CS); \
        PORTB = BM(FLASH_CS); \
        DDRC  = 0xFF; \
        PORTC = 0xFF; \
        DDRD  = BM(ASTAMP_LED2) | BM(ASTAMP_LED3) | BM(ASTAMP_LED4) | BM(CAN_TX) | BM(CAN_RS); \
        PORTD = BM(ASTAMP_LED2) | BM(ASTAMP_LED3) | BM(ASTAMP_LED4) | BM(CAN_RS); \
        DDRE  = BM(UART0_TXD) | BM(PWM_A) | BM(PWM_B) | BM(PWM_C); \
        PORTE = 0x00; \
    } while (0)

/********************************************************************************************************/
/****************************                      LEDS                     *****************************/
/********************************************************************************************************/

#define CLR_ASTAMP_LED2()			(PORTD |= BM(ASTAMP_LED2))
#define CLR_ASTAMP_LED3()			(PORTD |= BM(ASTAMP_LED3))
#define CLR_ASTAMP_LED4()			(PORTD |= BM(ASTAMP_LED4))

#define SET_ASTAMP_LED2()			(PORTD &= ~BM(ASTAMP_LED2))
#define SET_ASTAMP_LED3()			(PORTD &= ~BM(ASTAMP_LED3))
#define SET_ASTAMP_LED4()			(PORTD &= ~BM(ASTAMP_LED4))

#define TOGGLE_ASTAMP_LED2()		(PORTD ^= BM(ASTAMP_LED2))
#define TOGGLE_ASTAMP_LED3()		(PORTD ^= BM(ASTAMP_LED3))
#define TOGGLE_ASTAMP_LED4()		(PORTD ^= BM(ASTAMP_LED4))


#define CLEAR_LED7SEG()				do { PORTC = 0xFF; } while (0)
#define DISPLAY_LED7SEG(n)			do { PORTC = (n); } while (0)

/********************************************************************************************************/
/********************************************************************************************************/
/****************************                   INTERRUPTS                  *****************************/
/********************************************************************************************************/
/********************************************************************************************************/

/*****	Global Interrupt	*****************************************************************************/

#define ENABLE_GLOBAL_INT()         do { asm ("sei\n\t" ::); } while (0)
#define DISABLE_GLOBAL_INT()        do { asm ("cli\n\t" ::); } while (0)

/********************************************************************************************************/
/********************************************************************************************************/
/****************************     		      Timer0 Settings					*************************/
/********************************************************************************************************/
/********************************************************************************************************/

/*****	TCCR0 - Timer/Counter Control Register	*********************************************************/

/*****	TCCR0 Bit 7 - FOC0	*****************************************************************************/

#define FOC0_BM					0x80		// Force Output Compare (strobe)

/*****	TCCR0 Bits 3,6 - WGM01:0 Waveform Generation Mode	*********************************************/

#define WGM0_BM					0x48		// Mask for WGM01;WGM00 bits
#define WGM0_NORMAL				0x00		// Normal mode
#define WGM0_PHCPWM				0x40		// Phase Correct PWM mode
#define WGM0_CTC				0x08		// Clear Timer on Compare mode
#define WGM0_FPWM				0x48		// Fast PWM mode

/*****	TCCR0 Bit Field 5:4 - COM01:0 Compare Match Output Mode	*****************************************/

#define COM0_BM					0x30		// Bit Mask for COM01:0 bits
#define COM0_NORMAL				0x00		// Normal mode, OC0A disconnected
#define COM0_TOGGLE				0x10		// Toggle OC0A on compare match (not for PWM)
#define COM0_CLEAR				0x20		// Clear  OC0A on compare match
#define COM0_SET				0x30		// Set OC0A on compare match

/*****	TCCR0 Bit Field 2:0 - CS02:0 Clock Select (Prescaler)	*****************************************/

#define CS0_BM					0x07		// Bit Mask for CS02:0 bits
#define CS0_CLK_STOP			0x00		// Stop mode (the timer is not counting)
#define CS0_CLK_DIV1			0x01 		// Clock freq (no prescaling)
#define CS0_CLK_DIV8			0x02		// Clock freq / 8
#define CS0_CLK_DIV64			0x03		// Clock freq / 64
#define CS0_CLK_DIV256			0x04		// Clock freq / 256
#define CS0_CLK_DIV1024			0x05		// Clock freq / 1024
#define CS0_CLK_EXTCLKF			0x06		// Ext. clock on T2, falling edge
#define CS0_CLK_EXTCLKR			0x07		// Ext. clock on T2, rising edge

/********************************************************************************************************/
/********************************************************************************************************/
/*************************     		      Timer2 Settings					*************************/
/****************************************************************************************************/
/****************************************************************************************************/

/*****	TCCR2 - Timer/Counter Control Register	*****************************************************/

/*****	TCCR2 Bit 7 - FOC2	*************************************************************************/

#define FOC2_BM					0x80		// Force Output Compare (strobe)

/*****	TCCR2 Bits 3,6 - WGM21:0 Waveform Generation Mode	*****************************************/

#define WGM2_BM					0x48		// Mask for WGM21;WGM20 bits
#define WGM2_NORMAL				0x00		// Normal mode
#define WGM2_PHCPWM				0x40		// Phase Correct PWM mode
#define WGM2_CTC				0x08		// Clear Timer on Compare mode
#define WGM2_FPWM				0x48		// Fast PWM mode

/*****	TCCR2 Bit Field 5:4 - COM21:0 Compare Match Output Mode	*************************************/

#define COM2_BM					0x30		// Bit Mask for COM21:0 bits
#define COM2_NORMAL				0x00		// Normal mode, OC2A disconnected
#define COM2_TOGGLE				0x10		// Toggle OC2A on compare match (not for PWM)
#define COM2_CLEAR				0x20		// Clear  OC2A on compare match
#define COM2_SET				0x30		// Set OC2A on compare match

/*****	TCCR2 Bit Field 2:0 - CS22:0 Clock Select (Prescaler)	*************************************/

#define CS2_BM					0x07		// Bit Mask for CS22:0 bits
#define CS2_CLK_STOP			0x00		// Stop mode (the timer is not counting)
#define CS2_CLK_DIV1			0x01 		// Clock freq (no prescaling)
#define CS2_CLK_DIV8			0x02		// Clock freq / 8
#define CS2_CLK_DIV32			0x03		// Clock freq / 32
#define CS2_CLK_DIV64			0x04		// Clock freq / 64
#define CS2_CLK_DIV128			0x05		// Clock freq / 128
#define CS2_CLK_DIV256			0x06		// Clock freq / 256
#define CS2_CLK_DIV1024			0x07		// Clock freq / 1024

/****************************************************************************************************/
/*																									*/
/*	Macro:			TIMER2_INIT (div,cnt)															*/
/*																									*/
/*	Destription:	Initializing Timer2 for internal TIC timer:										*/
/*					Clar Timer on Compare match; OC0 disconnected; no PWM enabled;					*/
/*					Output Compare Match Interrupt Eanabled. An ISR function should be defined		*/
/*					in assocoation with Timer Compare Match event.									*/
/*																									*/
/*  Arguments:		div		unsigned char prescaler constant (CS2_CLK_xxx)							*/
/*  				cnt		unsigned char period counter											*/
/*																									*/
/****************************************************************************************************/

#define TIMER2_INIT(div,cnt) \
    do { \
        OCR2A = (cnt); \
	    TCCR2A = WGM2_CTC | (div); \
		TIMSK2 = BM(OCIE2A); \
    } while(0)

/********************************************************************************************************/
/********************************************************************************************************/
/****************************     		      Timer1/3 Settings					*************************/
/********************************************************************************************************/
/********************************************************************************************************/

/*****	TCCRnA - Timer/Counter Control Register	A	*****************************************************/

/*****	TCCRnA Bit Field 7:6 - COMnA1:0 Compare Match Output Mode for Channel A	*************************/

#define COMnA_BM				0xC0		// Bit Mask for COMnA1:0 bits
#define COMnA_NORMAL			0x00		// Normal mode, OCnA disconnected
#define COMnA_TOGGLE			0x40		// Toggle OCnA on compare match
#define COMnA_CLEAR				0x80		// Clear  OCnA on compare match
#define COMnA_SET				0xC0		// Set OCnA on compare match

/*****	TCCR1A Bit Field 5:4 - COM1B1:0 Compare Match Output Mode for Channel B	*************************/

#define COMnB_BM				0x30		// Bit Mask for COMnB1:0 bits
#define COMnB_NORMAL			0x00		// Normal mode, OCnB disconnected
#define COMnB_TOGGLE			0x10		// Toggle OCnB on compare match
#define COMnB_CLEAR				0x20		// Clear  OCnB on compare match
#define COMnB_SET				0x30		// Set OCnB on compare match

/*****	TCCRnA Bit Field 3:2 - COMnC1:0 Compare Match Output Mode for Channel C	*************************/

#define COMnC_BM				0x0C		// Bit Mask for COMnC1:0 bits
#define COMnC_NORMAL			0x00		// Normal mode, OCnC disconnected
#define COMnC_TOGGLE			0x04		// Toggle OCnC on compare match
#define COMnC_CLEAR				0x08		// Clear  OCnC on compare match
#define COMnC_SET				0x0C		// Set OCnC on compare match

/*****	TCCRnB - Timer/Counter Control Register	B	*****************************************************/

/*****	TCCRnB Bit 7 - ICNCn Input Capture Noise Canceler	*********************************************/

#define ICNCn_BM				0x80		// Switching Input Capture Noise Canceler ON

/*****	TCCRnB Bit 6 - ICESn Input Capture Edge Select	*************************************************/

#define ICESn_BM				0x40		// Input Capture Edge Select: 0 falling, 1 rising

/*****	TCCRnB Bit Field 2:0 - CSn2:0 Clock Select (Prescaler)	*****************************************/

#define CSn_BM					0x07		// Bit Mask for CSn2:0 bits
#define CSn_CLK_STOP			0x00		// Stop mode (the timer is not counting)
#define CSn_CLK_DIV1			0x01 		// Clock freq (no prescaling)
#define CSn_CLK_DIV8			0x02		// Clock freq / 8
#define CSn_CLK_DIV64			0x03		// Clock freq / 64
#define CSn_CLK_DIV256			0x04		// Clock freq / 256
#define CSn_CLK_DIV1024			0x05		// Clock freq / 1024
#define CSn_CLK_EXTCLKF			0x06		// Ext. clock on T1, falling edge
#define CSn_CLK_EXTCLKR			0x07		// Ext. clock on T1, rising edge

/*****	TCCRnB 4:3 & TCCRnA 1:0 - WGMn3:0 Waveform Generation Mode	*************************************/

#define WGMnA_BM				0x03		// Mask for TCCRnA;WGMn1:0 bits
#define WGMnB_BM				0x18		// Mask for TCCRnB;WGMn4:3 bits

#define WGMn_NORMAL				0x00		// Normal mode
#define WGMn_PHCPWM8			0x01		// PWM, Phase Correct, 8-bit
#define WGMn_PHCPWM9			0x02		// PWM, Phase Correct, 9-bit
#define WGMn_PHCPWM10			0x03		// PWM, Phase Correct, 10-bit
#define WGMn_CTCO				0x08		// CTC, OCRnA
#define WGMn_FPWM8				0x09		// Fast PWM, 8-bit
#define WGMn_FPWM9				0x0A		// Fast PWM, 9-bit
#define WGMn_FPWM10				0x0B		// Fast PWM, 10-bit
#define WGMn_PFCPWMI			0x10		// PWM, Phase and Frequency Correct, ICRn
#define WGMn_PFCPWMO			0x11		// PWM, Phase and Frequency Correct, OCRnA
#define WGMn_PHCPWMI			0x12		// PWM, Phase Correct, ICRn
#define WGMn_PHCPWMO			0x13		// PWM, Phase Correct, OCRnA
#define WGMn_CTCI				0x18		// CTC, ICRn
#define WGMn_FPWMI				0x1A		// Fast PWM, ICRn
#define WGMn_FPWMO				0x1B		// Fast PWM, OCRnA

/*****	TCCRnC - Timer/Counter Control Register	C	*****************************************************/

/*****	TCCRnC Bit 7:5 - FOCn Force Output Compare   ****************************************************/

#define FOCnA_BM				0x80		// Force Output Compare for Channel A (strobe)
#define FOCnB_BM				0x40		// Force Output Compare for Channel B (strobe)
#define FOCnC_BM				0x50		// Force Output Compare for Channel C (strobe)

/****************************************************************************************************/
/*																									*/
/*	Macro:			TIMER3_HOBBYPWM_INIT ()															*/
/*																									*/
/*	Destriction:	Initializing Timer3 for Hobby Servo PWM.										*/
/*					ICR = 19999	= 0x4E1F corresponding to f = 50Hz i.e. T = 20ms					*/
/*					-180 deg			0.5 ms	OCR =  500											*/
/*					-90  deg			1   ms	OCR = 1000											*/
/*					Neutral position	1.5 ms	OCR = 1500											*/
/*					+90  deg			2   ms	OCR = 2000											*/
/*					+180 deg			2.5 ms	OCR = 2500											*/
/*					span				2   ms        2000											*/
/*																									*/
/****************************************************************************************************/

#define TIMER3_HOBBYPWM_INIT() \
    do { \
        OCR3A = 0x05DC; \
        OCR3B = 0x05DC; \
        OCR3C = 0x05DC; \
        ICR3 = 0x4E1F; \
        TCCR3A = COMnA_CLEAR | COMnB_CLEAR | COMnC_CLEAR | (WGMn_FPWMI & WGMnA_BM); \
        TCCR3B = (WGMn_FPWMI & WGMnB_BM)| CSn_CLK_DIV8; \
    } while(0)
 
/****************************************************************************************************/
/*																									*/
/*	Macro:			PWM3m_SET_DUTY_CYCLE(dcyc)														*/
/*																									*/
/*	Destriction:	Setting the duty cycle of the PWM.												*/
/*																									*/
/*  Arguments:		dcyc		unsigned char duty cycle value - a 2-byte word						*/
/*																									*/
/****************************************************************************************************/

#define PWM3A_SET_DUTY_CYCLE(dcyc) do { OCR3A = (dcyc); } while (0)
#define PWM3B_SET_DUTY_CYCLE(dcyc) do { OCR3B = (dcyc); } while (0)
#define PWM3C_SET_DUTY_CYCLE(dcyc) do { OCR3C = (dcyc); } while (0)

/********************************************************************************************************/
/********************************************************************************************************/
/**************************                        UART                        **************************/
/********************************************************************************************************/
/********************************************************************************************************/

#define UCSZn_7BIT				0x04		// USART Character Size: 8-bit 
#define UCSZn_8BIT				0x06		// USART Character Size: 8-bit 

/********************************************************************************************************/
/*																										*/
/*	Macro:			INIT_UART0_IO()																		*/
/*																										*/
/*	Destription:	Initializing USART0 unit to asynchronous I/O communication.							*/
/*					Parameters: 8-bit, 1 stop-bit, no parity											*/
/*					Usage: programmed, no interrupts enabled											*/
/*																										*/
/*	Parameters:		(to be applied in the macro body)													*/
/*		 			Baud-rate (bps) at fosc = 8 MHz and U2X = 0 (normal speed):							*/
/*					Baud-rate =  2400	UBRR = 207	Error = 0.2%										*/
/*					Baud-rate =  4800	UBRR = 103	Error = 0.2%										*/
/*					Baud-rate =  9600	UBRR =  51	Error = 0.2%										*/
/*					Baud-rate = 19200	UBRR =  25	Error = 0.2%										*/
/*																										*/
/********************************************************************************************************/

#define INIT_UART0() \
			do { \
				 UBRR0 = 25; \
				 UCSR0A = BM(TXC0); \
				 UCSR0B = BM(RXEN0) | BM(TXEN0); \
				 UCSR0C = UCSZn_8BIT; \
			   } while (0)

#define PUTC_UART0(ch) \
			do { \
				 while ((UCSR0A & BM(UDRE0)) == 0) continue; \
				 UDR0 = (ch); \
			   } while (0)

#define GETC_UART0(ch) \
			do { \
				 ch = UDR0; \
			   } while (0)

#define CHECK_UART0() ((UCSR0A & BM(RXC0)) != 0)

/********************************************************************************************************/
/*																										*/
/*	Macro:			INIT_UART1_IO()																		*/
/*																										*/
/*	Destription:	Initializing USART0 unit to asynchronous I/O communication.							*/
/*					Parameters: 8-bit, 1 stop-bit, no parity											*/
/*					Usage: programmed, no interrupts enabled											*/
/*																										*/
/*	Parameters:		(to be applied in the macro body)													*/
/*		 			Baud-rate (bps) at fosc = 8 MHz and U2X = 0 (normal speed):							*/
/*					Baud-rate =  2400	UBRR = 207	Error = 0.2%										*/
/*					Baud-rate =  4800	UBRR = 103	Error = 0.2%										*/
/*					Baud-rate =  9600	UBRR =  51	Error = 0.2%										*/
/*					Baud-rate = 19200	UBRR =  25	Error = 0.2%										*/
/*					Baud-rate = 38400	UBRR =  12	Error = 0.2%										*/
/*																										*/
/********************************************************************************************************/

#define INIT_UART1() \
			do { \
				 UBRR1 = 25; \
				 UCSR1A = BM(TXC1); \
				 UCSR1B = BM(RXEN1) | BM(TXEN1); \
				 UCSR1C = UCSZn_8BIT; \
			   } while (0)

#define PUTC_UART1(ch) \
			do { \
				 while ((UCSR1A & BM(UDRE1)) == 0) continue; \
				 UDR1 = (ch); \
			   } while (0)

#define GETC_UART1(ch) \
			do { \
				 ch = UDR1; \
			   } while (0)

#define CHECK_UART1() ((UCSR1A & BM(RXC1)) != 0)


/********************************************************************************************************/
/********************************************************************************************************/
/**************************                         ADC                        **************************/
/********************************************************************************************************/
/********************************************************************************************************/

/*****	ADMUX Bit Field 7:6 - REFS1:0 Reference Selection Bits	*****************************************/

#define REFS_BM					0xC0		// Bit Mask for REFS2:0 bits
#define REFS_AREF				0x00		// External reference connected to AREF
#define REFS_AVCC				0x40		// AVCC with external capacitor at AREF
#define REFS_2V56				0xC0		// Internal 2.56 V reference with external capacitor at AREF

/*****	ADMUX Bit Field 4:0 - MUX4:0 Analog Channel and Gain Selection Bits	*****************************/

#define MUX_BM					0x1F		// Bit Mask for REFS2:0 bits
#define MUX_ADC0				0x00		// Single ended ADC0
#define MUX_ADC1				0x01		// Single ended ADC1
#define MUX_ADC2				0x02		// Single ended ADC2
#define MUX_ADC3				0x03		// Single ended ADC3
#define MUX_ADC4				0x04		// Single ended ADC4
#define MUX_ADC5				0x05		// Single ended ADC5
#define MUX_ADC6				0x06		// Single ended ADC6
#define MUX_ADC7				0x07		// Single ended ADC7
#define MUX_ADC0_ADC0_10X		0x08		// Differential ADC0 - ADC0 Gain 10
#define MUX_ADC1_ADC0_10X		0x09		// Differential ADC1 - ADC0 Gain 10
#define MUX_ADC0_ADC0_200X		0x0A		// Differential ADC0 - ADC0 Gain 200
#define MUX_ADC1_ADC0_200X		0x0B		// Differential ADC1 - ADC0 Gain 200
#define MUX_ADC2_ADC2_10X		0x0C		// Differential ADC2 - ADC2 Gain 10
#define MUX_ADC3_ADC2_10X		0x0D		// Differential ADC3 - ADC2 Gain 10
#define MUX_ADC2_ADC2_200X		0x0E		// Differential ADC2 - ADC2 Gain 200
#define MUX_ADC3_ADC2_200X		0x0F		// Differential ADC3 - ADC2 Gain 200
#define MUX_ADC0_ADC1			0x10		// Differential ADC0 - ADC1 Gain 1
#define MUX_ADC1_ADC1			0x11		// Differential ADC1 - ADC1 Gain 1
#define MUX_ADC2_ADC1			0x12		// Differential ADC2 - ADC1 Gain 1
#define MUX_ADC3_ADC1			0x13		// Differential ADC3 - ADC1 Gain 1
#define MUX_ADC4_ADC1			0x14		// Differential ADC4 - ADC1 Gain 1
#define MUX_ADC5_ADC1			0x15		// Differential ADC5 - ADC1 Gain 1
#define MUX_ADC6_ADC1			0x16		// Differential ADC6 - ADC1 Gain 1
#define MUX_ADC7_ADC1			0x17		// Differential ADC7 - ADC1 Gain 1
#define MUX_ADC0_ADC2			0x18		// Differential ADC0 - ADC2 Gain 1
#define MUX_ADC1_ADC2			0x19		// Differential ADC1 - ADC2 Gain 1
#define MUX_ADC2_ADC2			0x1A		// Differential ADC2 - ADC2 Gain 1
#define MUX_ADC3_ADC2			0x1B		// Differential ADC3 - ADC2 Gain 1
#define MUX_ADC4_ADC2			0x1C		// Differential ADC4 - ADC2 Gain 1
#define MUX_ADC5_ADC2			0x1D		// Differential ADC5 - ADC2 Gain 1
#define MUX_1V23_VBG			0x1E		// 1.25 V Bandgap Voltage Reference
#define MUX_0V_GND				0x1F		// 0 V Ground

/*****	ADCSRA Bit Field 2:0 - ADPS2:0 ADC Prescaler Selection	*****************************************/

#define ADPS_BM					0x07		// Bit Mask for ADPS2:0 bits
#define ADPS_CLK_BASE			0x00		// Clock freq / 2
#define ADPS_CLK_DIV2			0x01 		// Clock freq / 2
#define ADPS_CLK_DIV4			0x02		// Clock freq / 4
#define ADPS_CLK_DIV8			0x03		// Clock freq / 8
#define ADPS_CLK_DIV16			0x04		// Clock freq / 16
#define ADPS_CLK_DIV32			0x05		// Clock freq / 32
#define ADPS_CLK_DIV64			0x06		// Clock freq / 64
#define ADPS_CLK_DIV128			0x07		// Clock freq / 128

/********************************************************************************************************/
/*																										*/
/*	Macro:			INIT_ADC ()																			*/
/*																										*/
/*	Destription:	Initializing the Analog To Digital Converter unit,									*/
/*					Voltage Reference:		AVCC														*/
/*					Adjustment of Result: 	right														*/
/*					Channel and Gain:		single ended ADC0											*/
/*					ADC Enable: 			on															*/
/*					ADC Free Run:			off															*/
/*					ADC Interrupt Enable:	off															*/
/*					ADC Prescaler:			128															*/
/*																										*/
/********************************************************************************************************/

#define INIT_ADC() \
			do { \
				 ADMUX = REFS_AVCC; \
				 ADCSRA = BM(ADEN) | BM(ADIE) | ADPS_CLK_DIV128; \
			   } while (0)

/********************************************************************************************************/
/*																										*/
/*	Macro:			SELECT_ADC_CHANNEL (ch)																*/
/*																										*/
/*	Arguments:		char ch			channel selection constant											*/
/*																										*/
/*	Destription:	Selecting channel and gain for the ADC.												*/
/*																										*/
/********************************************************************************************************/

#define SELECT_ADC_CHANNEL(ch) (ADMUX = (ADMUX & ~MUX_BM) | (ch))

/********************************************************************************************************/
/*																										*/
/*	Macro:			START_AD_CONVERSION ()																*/
/*																										*/
/*	Destription:	Starting AD Conversion.																*/
/*																										*/
/********************************************************************************************************/

#define START_AD_CONVERSION() (ADCSRA |= BM(ADSC) | BM(ADIF))

/********************************************************************************************************/
/*																										*/
/*	Macro:			GET_ADC_RESULT (value)																*/
/*																										*/
/*	Arguments:		unsigned int value		variable to store the value									*/
/*																										*/
/*	Destription:	Getting the result of the AD Conversion.											*/
/*																										*/
/********************************************************************************************************/

#define GET_ADC_RESULT(value) (value = ADC)

/********************************************************************************************************/

#endif
