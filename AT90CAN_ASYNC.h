#ifndef AT90_CAN_ASYNC_H
#define AT90_CAN_ASYNC_H

/****************************************************************************************************/
/* Include files																					*/
/****************************************************************************************************/

#include "include.h"
#include "can.h"
#include "assert.h"
#include "String_MPC_Robot.h"
#define F_CPU 8000000UL
#include <util/delay.h>
#include <stdlib.h>

/****************************************************************************************************/
/*****	Enums																					*****/
/****************************************************************************************************/

enum INIT_STATUS{
	no_init,
	init_1,
	init_2,
	running
};

enum OPERATION_STATE{
	init,
	primitive,
	controller,
	config,
	bluetooth
};

enum CONFIG_STATE{
	reset_incntrs
};

enum CONTROLLER_STATE{
	hold,
	input,
	accepted,
	trajectory
};

/****************************************************************************************************/
/*****	Defines																					*****/
/****************************************************************************************************/

#define sbi(a, b) a |= (1 << (b))
#define cbi(a, b) a &= ~(1 << (b))

/** Primitive mode button defines */
#define POSITIVE_BUTTON_1 1
#define NEGATIVE_BUTTON_1 3
#define POSITIVE_BUTTON_2 4
#define NEGATIVE_BUTTON_2 6
#define STOP_BUTTON 2


/** Operation mode change button defines */
#define NEXT_MODE_BUTTON 12
#define PREV_MODE_BUTTON 10

/** Controller mode button defines */
#define ENTER_INPUT_MODE_BUTTON 2
#define FINISH_INPUT_BUTTON 10
#define CHANGE_SIGN_BUTTON 12
#define START_TRAJECTORY_BUTTON 2

#define MAX_DIGITS_NUM 5

/** Config mode button defines */
#define NEXT_CONFIG_STATE_BUTTON 3
#define PREV_CONFIG_STATE_BUTTON 1
#define RESET_INCNTR_REQUEST_BUTTON 2

/** CAN message defines */
#define PRIMITIVE_MODE_REQUEST 0x01
#define CONTROLLER_MODE_REQUEST 0x02
#define CONFIG_MODE_REQUEST 0x06
#define BLUETOOTH_MODE_REQUEST 0x08
#define START_TRAJECTORY_REQUEST 0x09
#define EMPTY_REQUEST 0xFF

#define PRIMITIVE_MODE_ACK 0x01
#define CONTROLLER_MODE_ACK 0x02
#define CONFIG_MODE_ACK 0x03
#define BLUETOOTH_MODE_ACK 0x04


#define CNTR1_RESET_REQUEST 0x03 
#define CNTR2_RESET_REQUEST 0x04
#define CNTR3_RESET_REQUEST 0x05
#define ALL_CNTR_RESET_REQUEST 0x07

#define INVGEO_OK_MESSAGE 0x01
#define INVGEO_ERROR_MESSAGE 0x02
#define ON_TRAJECTORY_MESSAGE 0x03
#define CONTROLLER_HOLD_MESSAGE 0x04


/****************************************************************************************************/
/*****	Function prototypes																		*****/
/****************************************************************************************************/

void incoming_init(CAN_packet* p, unsigned char mob);
void incoming_incremental_out_value(CAN_packet* p, unsigned char mob);
void incoming_incremental_in_value(CAN_packet* p, unsigned char mob);
void incoming_operation_mode_changed(CAN_packet* p, unsigned char mob);
void incoming_controller_message(CAN_packet* p, unsigned char mob);

void system_init();
void UART_init();
void UART_BT_transmit(U8 data);

void UART_RTS_low();
void UART_RTS_high();
U8 poll_UART_CTS();
void enable_BT_module();
void disable_BT_module();

void padstr(char* buffer, char* buf);

void LCD_RS_high();
void LCD_RS_low();
void LCD_RW_high();
void LCD_RW_low();
void LCD_E_high();
void LCD_E_low();
void LCD_DB(U8 data);
void LCD_comm(U8 RS, U8 RW, U8 comm);
void LCD_clear();
void LCD_home();
void LCD_init();
void LCD_text();

void green_LED_on();
void green_LED_off();
void green_LED_toggle();
void yellow_LED_on();
void yellow_LED_off();
void yellow_LED_toggle();
void red_LED_on();
void red_LED_off();
void red_LED_toggle();

void request_incremental_counter_reset();

void prepare_input_coords_packet();



#endif 
