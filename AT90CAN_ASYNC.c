/****************************************************************************************************/
/*																									*/
/* AT90CAN_SYNC.C																					*/
/*																									*/
/* CAN - SPI peripherals transceiver firmware for MPC555 based robot controller						*/
/*																									*/
/* Date:	March 20, 2012																			*/
/*																									*/
/* Author: Opra Balazs																				*/
/*																									*/
/* Based on: Atmel CAN spy echo example																*/
/*																									*/
/****************************************************************************************************/
/* Compiler: AVR-GCC																				*/
/* Target platform: MPC555 robot controller															*/
/****************************************************************************************************/

/****************************************************************************************************/
/* Include files																					*/
/****************************************************************************************************/


#include "AT90CAN_ASYNC.h"

/****************************************************************************************************/
/*****  Global variables 																		*****/
/****************************************************************************************************/

volatile int16_t input_coords[2];
volatile U8 invgeo_error;
volatile U8 keys_value;
volatile U8 digits_input;
volatile U8 coord_edited;
volatile U8 keys;
volatile U8 previous_keys;
volatile char digit_buf[2];
volatile char buf[80]; //buffer for string operations
volatile char value_string[5]; //buffer for string converted from U16 format
CAN_packet init_ack_async_packet;
CAN_packet stop_button_packet;
CAN_packet primitive_button_packet;
CAN_packet async_request_packet;
CAN_packet input_coords_packet;
//menu_sates menu_state;
volatile enum INIT_STATUS init_status;
volatile enum OPERATION_STATE operation_state;
volatile enum OPERATION_STATE previous_operation_state;
volatile enum CONFIG_STATE config_state;
volatile enum CONFIG_STATE previous_config_state;
volatile enum CONTROLLER_STATE controller_state;
volatile enum CONTROLLER_STATE previous_controller_state;
//Bluetooth to can package construction
volatile U8 bt_package_flag;//if [1,2] we receive the X coordinates, if [3,4] then the Y
volatile int16_t bt_coord_buffer;
/****************************************************************************************************/
/*****  Function prototypes																		*****/
/****************************************************************************************************/

ISR(TIMER1_COMPA_vect);

/****************************************************************************************************/
/*****  Function definitions																	*****/
/****************************************************************************************************/

/********************
main function
***********************/


int main(){
	char first_line[40];
	char second_line[40];
	U8 i;
	//menu_states menu_state;
	operation_state = init;
	previous_operation_state = init;
	config_state = previous_config_state = reset_incntrs;
	init_status = no_init;
	keys = previous_keys = 0;	
	bt_package_flag = 0;
	bt_coord_buffer = 0;
	
	//prepare the init_ack_async_packet
	init_ack_async_packet.id = 0x03;
	init_ack_async_packet.length = 1;
	init_ack_async_packet.data[0] = 0x15;

	//prepare the stop_button_packet
	stop_button_packet.id = 0x04;
	stop_button_packet.length = 1;
	stop_button_packet.data[0] = 0x01;

	//prepare the primitive_button_packet
	primitive_button_packet.id = 0x08;
	primitive_button_packet.length = 1;
	primitive_button_packet.data[0] = 0x00;

	//prepare the async_request_packet
	async_request_packet.id = 0x0C;
	async_request_packet.length = 1;
	async_request_packet.data[0] = 0x00;

	//prepare the input_coords_packet
	input_coords_packet.id = 0x0F;
	input_coords_packet.length = 4;
	for (i = 0; i < input_coords_packet.length; i ++) input_coords_packet.data[i] = 0;


	system_init();
	reset_string();	

	for (i=0; i<80; i++){
		first_line[i] = 0x00;	
		second_line[i] = 0x00;
	}
	

	//global interrupt enable
	sei();
	if (!add_string("Initializing...",0)) red_LED_on;
	LCD_text();
	
	green_LED_on();

	if (prepare_rx(1,0x01,0xFF,incoming_init)) red_LED_on();  //debug for incoming message
	
//	UART_BT_transmit('A');

	while(1){
		if (init_status == init_2) {
			LCD_text();
		} else if (init_status == running) {
			switch (operation_state) {
			/////////////////// PRIMITIVE MODE ///////////////////////
			case primitive:
				if (previous_operation_state != operation_state) {
					reset_string();
					add_string("- Direct operation -", LINE_1);
					add_string("S1:      S2:", LINE_3);
					add_string("<-* Change mode  #->", LINE_4);
					previous_operation_state = operation_state;

					disable_BT_module();
				}
				if (keys != previous_keys) {
					switch (keys) {
						case STOP_BUTTON:
							while(!can_tx(2, &stop_button_packet));
							reset_line_2();
							add_string("Stopped", LINE_2);
							break;
						case POSITIVE_BUTTON_1:
							primitive_button_packet.data[0] = 0x01;
							while(!can_tx(2, &primitive_button_packet));
							reset_line_2();
							add_string("Seg 1 pos movement", LINE_2);
							break;
						case POSITIVE_BUTTON_2:
							primitive_button_packet.data[0] = 0x02;
							while(!can_tx(2, &primitive_button_packet));
							reset_line_2();
							add_string("Seg 2 pos movement", LINE_2);
							break;
						case NEGATIVE_BUTTON_1:
							primitive_button_packet.data[0] = 0x03;
							while(!can_tx(2, &primitive_button_packet));
							reset_line_2();
							add_string("Seg 1 neg movement", LINE_2);
							break;
						case NEGATIVE_BUTTON_2:
							primitive_button_packet.data[0] = 0x04;
							while(!can_tx(2, &primitive_button_packet));
							reset_line_2();
							add_string("Seg 2 neg movement", LINE_2);
							break;
						case 0:
							if ((previous_keys == POSITIVE_BUTTON_1) ||
								(previous_keys == POSITIVE_BUTTON_2) ||
								(previous_keys == NEGATIVE_BUTTON_1) ||
								(previous_keys == NEGATIVE_BUTTON_2)) {

								primitive_button_packet.data[0] = 0x05;
								while(!can_tx(2, &primitive_button_packet));
								reset_line_2();
								add_string("No movement", LINE_2);
							}
							break;
						case PREV_MODE_BUTTON:
							async_request_packet.data[0] = CONFIG_MODE_REQUEST;
							while(!can_tx(6, &async_request_packet));
							break;
						case NEXT_MODE_BUTTON:
							async_request_packet.data[0] = CONTROLLER_MODE_REQUEST;
							while(!can_tx(6, &async_request_packet));
							break;
						default: break;
					}
					previous_keys = keys;
				}
				break;
			/////////////////// CONTROLLER MODE ///////////////////////
			case controller:
				if (previous_operation_state != operation_state) {
					reset_string();
					add_string("-  Controller on   -", LINE_1);
					add_string(" (2) - Input coords ", LINE_2);
					reset_line_3();				
					add_string("<-* Change mode  #->", LINE_4);
					previous_operation_state = operation_state;
					controller_state = previous_controller_state = hold;

					disable_BT_module();
					invgeo_error = 0;
				}
				if (previous_controller_state != controller_state) {
					switch (controller_state) {
						case input:
							reset_line_2();
							add_string("Enter input!(* - OK)", LINE_2);
							reset_line_3();
							add_string("X:      Y:", LINE_3);
							digits_input = 0;
							input_coords[0] = 0;
							input_coords[1] = 0;
							coord_edited = 0;
							break;
						case hold:
							reset_line_2();
							reset_line_3();
							add_string(" (2) - Input coords ", LINE_2);
							if (invgeo_error) add_string("   Invgeo error!    ", LINE_3);
							break;
						case accepted:
							invgeo_error = 0;
							reset_line_2();
							reset_line_3();
							add_string("Invgeo OK! (2)-Start", LINE_2);
							break;
						case trajectory:
							reset_line_2();
							reset_line_3();
							add_string("Following trajectory", LINE_2);
							break;
						default: break;
					}
					previous_controller_state = controller_state;
				}
				if (keys != previous_keys) {
					cli();
					switch (controller_state) {
						case hold:
							switch (keys) {
								case ENTER_INPUT_MODE_BUTTON:
									controller_state = input;
									break;
								case PREV_MODE_BUTTON:
									async_request_packet.data[0] = PRIMITIVE_MODE_REQUEST;
									while(!can_tx(6, &async_request_packet));
									break;
								case NEXT_MODE_BUTTON:
									async_request_packet.data[0] = BLUETOOTH_MODE_REQUEST;
									while(!can_tx(6, &async_request_packet));
									break;								
								default: break;
							}
							break;
						case input:
							switch (keys) {
								case FINISH_INPUT_BUTTON:
									digits_input = 0;
									if (coord_edited == 0) coord_edited++;
									else {
										controller_state = hold;
										prepare_input_coords_packet();
										while(!can_tx(8, &input_coords_packet));
									}
									break;
								case CHANGE_SIGN_BUTTON:
									input_coords[coord_edited] = input_coords[coord_edited] * (-1);
									if (input_coords[coord_edited] >= 0) add_string(" ", LINE_3 + 2 + coord_edited * 8);
									else add_string("-",  LINE_3 + 2 + coord_edited * 8);
									break;
								default:
									if (keys == 0) break;
									keys_value = keys;
									if (keys_value == 11) keys_value = 0;								

									if ((input_coords[coord_edited] == 0) && (digits_input == 1)) {
										if (keys_value == 0) break;
										else --digits_input;
									}
									

									input_coords[coord_edited] = input_coords[coord_edited] * 10 + keys_value;
									keys_value += 48;
									digit_buf[0] = keys_value;
									digit_buf[1] = 0;
									add_string(digit_buf, LINE_3 + 3 + coord_edited * 8 + digits_input);

									if (++digits_input == MAX_DIGITS_NUM) {
										digits_input = 0;
										if (coord_edited == 0) coord_edited++;
										else {
											controller_state = hold;		
											prepare_input_coords_packet();
											while(!can_tx(8, &input_coords_packet));
										}
									}
									break;
							}
							break;
						case accepted:
							switch (keys) {
								case START_TRAJECTORY_BUTTON:
									async_request_packet.data[0] = START_TRAJECTORY_REQUEST;
									while(!can_tx(6, &async_request_packet));
									async_request_packet.data[0] = EMPTY_REQUEST;
									while(!can_tx(6, &async_request_packet));
									controller_state = hold;
									break;
								case PREV_MODE_BUTTON:
									async_request_packet.data[0] = PRIMITIVE_MODE_REQUEST;
									while(!can_tx(6, &async_request_packet));
									break;
								case NEXT_MODE_BUTTON:
									async_request_packet.data[0] = BLUETOOTH_MODE_REQUEST;
									while(!can_tx(6, &async_request_packet));
									break;								
								default: break;
							}
							break;
						default: break;
					}
					previous_keys = keys;
					sei();
				}
				break;
			/////////////////// BLUETOOTH MODE ///////////////////////
			case bluetooth:
				if( bt_package_flag == 5 ){	//There are no partial coordinates
					//Updating the coordinates
					controller_state = hold;
					prepare_input_coords_packet();
					while(!can_tx(8, &input_coords_packet));
					
					//Updating the trajectory
					async_request_packet.data[0] = START_TRAJECTORY_REQUEST;
					while(!can_tx(6, &async_request_packet));
					async_request_packet.data[0] = EMPTY_REQUEST;
					while(!can_tx(6, &async_request_packet));
					controller_state = hold;
					
					bt_package_flag = 0;
				}
				if (previous_operation_state != operation_state) {
					reset_string();
					add_string("-  Bluetooth mode  -", LINE_1);
					reset_line_2();				
					add_string("<-* Change mode  #->", LINE_4);
					previous_operation_state = operation_state;
					invgeo_error = 0;
					// Enable bluetooth module
					enable_BT_module();
					
				} 
				if (previous_controller_state != controller_state) {
					switch (controller_state) {
						case input:
						break;
						case hold:
							reset_line_2();
							reset_line_3();
							if (invgeo_error) add_string("   Invgeo error!    ", LINE_3);
						break;
						case accepted:
							invgeo_error = 0;
							reset_line_2();
							reset_line_3();
							add_string("Invgeo OK!", LINE_2);
						break;
						case trajectory:
							reset_line_2();
							reset_line_3();
							add_string("Following trajectory", LINE_2);
						break;
						default: break;
					}
					previous_controller_state = controller_state;
				}
				if (keys != previous_keys) {
					switch (keys) {
						case PREV_MODE_BUTTON:
							async_request_packet.data[0] = CONTROLLER_MODE_REQUEST;
							while(!can_tx(6, &async_request_packet));
							break;
						case NEXT_MODE_BUTTON:
							async_request_packet.data[0] = CONFIG_MODE_REQUEST;
							while(!can_tx(6, &async_request_packet));
							break;
						default: break;
					}
					previous_keys = keys;
				}
				break;
			/////////////////// CONFIG MODE ///////////////////////
			case config:
				if (previous_operation_state != operation_state) {
					reset_string();
					add_string("-   Config mode    -", LINE_1);

					switch(config_state) {
						case reset_incntrs:
							reset_line_2();
							add_string("<-1 Reset cntrs  3->", LINE_2);
							break;
						default: break;
					}

					add_string("S1:      S2:", LINE_3);
					add_string("<-* Change mode  #->", LINE_4);
					previous_operation_state = operation_state;

					disable_BT_module();
				}
				
				if (keys != previous_keys) {
					switch (keys) {
						case PREV_CONFIG_STATE_BUTTON:
							break;
						case NEXT_CONFIG_STATE_BUTTON:
							break;
						case RESET_INCNTR_REQUEST_BUTTON:
							request_incremental_counter_reset();
							async_request_packet.data[0] = EMPTY_REQUEST;
							while(!can_tx(6, &async_request_packet));
							break;
						case PREV_MODE_BUTTON:
							async_request_packet.data[0] = BLUETOOTH_MODE_REQUEST;
							while(!can_tx(6, &async_request_packet));
							break;
						case NEXT_MODE_BUTTON:
							async_request_packet.data[0] = PRIMITIVE_MODE_REQUEST;
							while(!can_tx(6, &async_request_packet));
							break;
						default: break;
					}
					previous_keys = keys;
				}
				break;
			default:
				break;
			}
			LCD_text();
		} else if (init_status == init_1) {
			reset_string();
			add_string("MPC init OK!", LINE_1);
			LCD_text();
		}
	}

	return 0;
}



/********************
void system init
initialises the controller for communication with the LCD and keyboard
***********************/

void system_init(){
	
	DDRE = 0xEE; //port E 7-5 are outputs for LCD, other pins are set for Bluetooth module
	DDRA = 0xFF; //all pins of port A are outputs
	DDRF = 0x0E; //keyboard columns are outputs
	PORTF = 0x0F; //keyboard columns are set high
	PORTC = 0xE0; //turn on pullup resistors on keyboard columns
	DDRC = 0x0F; //LEDs and the Bluetooth reset pin are outputs
	DDRD |= 0xA0; //SN65HVD enable signal is and CANTX are outputs
	PORTD &= 0x7F; //SN65HVD enable signal is low

	keys = 0x0000;
		
	TCCR1B |= (1 << WGM12);	//Timer 1 CTC mode, TOP is in OCR1A
	OCR1A = 1250;	//overflow occurs with 100Hz
	TIMSK1 = 1 << OCIE1A; //enable Timer channel A output compare interrupt
	/*
	init_CAN();
	*/
	LCD_init();
	can_init();
	UART_init();
	

	TCCR1B |= (1 << CS11)|(1 << CS10); //start timer
	

	
}


/** Initializes the UART module for communication with the BTM443 Bluetooth module */

void UART_init(){
	disable_BT_module();
	UCSR0A = 0x00;
	UBRR0H = 0x00;
	UBRR0L = 0x03; 	// Baud rate is set to 115k (assuming fclkio = 8MHz)
	UCSR0C = (1 << UCSZ01) | (1 <<UCSZ00); // 8-N-1 UART frame
	UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0); // Enables the receiver interrupt
}


/** Transmits the data, no handshaking included
 \param U8 data The data to be transmitted
 */
void UART_BT_transmit(U8 data){
	while ( ! ( UCSR0A & (1<<UDRE0)));
	UDR0 = data;
}

/************************
UART pin handling functions
*************************/

void UART_RTS_low(){
	PORTE &= ~0x04;
}

void UART_RTS_high(){
	PORTE |= 0x04;
}

U8 poll_UART_CTS(){
	if (PORTE & 0x10) return 1;
	return 0;
}


/** Enables the bluetooth module by driving RESET high */

void enable_BT_module() {
	PORTC |= 0x01;			// Set the RESET input of the Bluetooth module high
}

/** Disables the bluetooth module by driving RESET low */
void disable_BT_module() {
	PORTC &= ~(0x01);		// Set the RESET input of the Bluetooth module low
}

/********************
LCD pin handling functions
***********************/

void LCD_RS_high(){
	PORTE |= 0x20;
}

void LCD_RS_low(){
	PORTE &= 0xDF;
}

void LCD_RW_high(){
	PORTE |= 0x40;
}

void LCD_RW_low(){
	PORTE &= 0xBF;
}

void LCD_E_high(){
	PORTE |= 0x80;
}

void LCD_E_low(){
	PORTE &= 0x7F;
}

void LCD_DB(U8 data){
	PORTA = data;
}

/********************
LCD_comm
sends the command in COMM
***********************/

void LCD_comm(U8 RS, U8 RW, U8 comm){
	if (RS) LCD_RS_high();
	else LCD_RS_low();
	if (RW) LCD_RW_high();
	else LCD_RW_low();
	
	LCD_E_high();
	LCD_DB(comm);
	_delay_ms(0.1d);
	LCD_E_low();
	_delay_ms(0.1d);
}
	


/********************
LCD_clear()
clears the screen
***********************/

void LCD_clear(){
	LCD_comm(0,0,0x01);
	_delay_ms(2.0d);
}

/********************
LCD_return()
returns the cursor home
***********************/

void LCD_home(){
	LCD_comm(0,0,0x02);
	_delay_ms(2.0d);
}


/********************
LCD_init()
initalises the LCD for operation
***********************/

void LCD_init(){
	
	//waits 40ms (required by LCD after power on) //all wait values are doubled then increased tenfold for debugging!
	_delay_ms(200.0d);
	LCD_comm(0,0,0x30);
	_delay_ms(4.2d);
	LCD_comm(0,0,0x30);
	_delay_ms(1.0d);
	LCD_comm(0,0,0x30);
	//set 8 bit interface, 2 line, wide char
	_delay_ms(1.0d);
	LCD_comm(0,0,0x3C);
	_delay_ms(1.0d);
	//display off
	LCD_comm(0,0,0x08);
	_delay_ms(1.0d);
	LCD_clear();
	//sets entry mode
	LCD_comm(0,0,0x06);
	_delay_ms(1.0d);
	//displan on
	LCD_comm(0,0,0x0C);
	_delay_ms(1.0d);
}

/********************
LCD_text()
writes text to display, line == 1 ->first line, line == 2, second line
***********************/

void LCD_text(){
	U8 i;
	char bufr[40];
	
	for (i=0; i<20; i++) {
		bufr[i] = __string_buffer[i];
		bufr[i+20] = __string_buffer[i+40];
	}
	LCD_comm(0,0,0x80);
	_delay_ms(0.02d);
	for(i=0; i<40; i++){
		LCD_comm(1,0,bufr[i]);
	//	_delay_ms(0.02d);
	}
	for(i=0; i<20; i++){
		bufr[i] = __string_buffer[i+20];
		bufr[i+20] = __string_buffer[i+60];
	}
	LCD_comm(0,0,0xC0);
	for(i=0; i<40; i++){
		LCD_comm(1,0,bufr[i]);
	//	_delay_ms(0.02d);
	}

}

/********************
padstr()
Needs a null terminated input string (buffer) and a 80 character long char array(buf).
Pads the string it with whitespaces
to be 80 characters long, clips it if it's longer and 
puts it into the array.
***********************/

void padstr(char* buffer, char* buf){
	U8 i=0;
	while ((buffer[i] != '\0') && i<80){
		buf[i] = buffer[i];
		i++;
	}
	for(; i<80; i++) buf[i] = ' ';
}


/** Interrupt routine for USART0 RX complete */ 

ISR(USART0_RX_vect) {
	volatile U8 data = UDR0;
	static int16_t last_X = 0;
	static int16_t last_Y = 0;
	
	if (operation_state != bluetooth) return;
	
	switch(bt_package_flag){
		case 0://frame sync
			if ( data == (U8)"X" ) bt_package_flag++;
		break;
		case 1://X_H
			bt_coord_buffer = data << 8;
			bt_package_flag++;
		break;
		case 2://X_L
			bt_coord_buffer |= data;
			if(invgeo_error){
				input_coords[0] -= last_X;
			}
			last_X = bt_coord_buffer;
			input_coords[0] += bt_coord_buffer;
				
			bt_package_flag++;
		break;
		case 3://Y_H
			bt_coord_buffer = data << 8;
			bt_package_flag++;
		break;
		case 4://Y_L
			bt_coord_buffer |= data;
			if(invgeo_error){
				input_coords[1] -= last_Y;
			}
			last_X = bt_coord_buffer;
			input_coords[1] += bt_coord_buffer;
			
			bt_package_flag++; 
		break;
		case 5:
		default:
		break;
	}
	
	TWBR = data;	
	UART_BT_transmit(data);
	add_string(data, LINE_2);
}

/********************
Interrupt fuction which is called on Timer 1 A channel compare (every 100ms)
Currently handles the keyboard
***********************/

ISR(TIMER1_COMPA_vect){
	volatile U8 pressed = 0;
//	volatile char num[2];
	//num[1] = 0;
	NOP();

	cli();
	PINF = 0x02; //set first keyboard column low
	_delay_ms(0.01d);  //wait for propagation delay
	if (!(PINF & 0x01)) pressed = 10; //check if each row is pressed
	if (!(PINC & 0x80)) pressed = 7;
	if (!(PINC & 0x40)) pressed = 4;
	if (!(PINC & 0x20)) pressed = 1;
	PINF = 0x02; //set first keyboard column high

	PINF = 0x04; //set second keyboard column low
	_delay_ms(0.01d);	//wait for propagation delay
	if (!(PINF & 0x01)) pressed = 11; //check if each row is pressed
	if (!(PINC & 0x80)) pressed = 8;
	if (!(PINC & 0x40)) pressed = 5;
	if (!(PINC & 0x20)) pressed = 2;
	PINF = 0x04; //set second keyboard column high

	PINF = 0x08; //set third keyboard column high
	_delay_ms(0.01d);	//wait for propagation delay
	
	if (!(PINF & 0x01)) pressed = 12; //check if each row is pressed
	if (!(PINC & 0x80)) pressed = 9;
	if (!(PINC & 0x40)) pressed = 6;
	if (!(PINC & 0x20)) pressed = 3;
	PINF = 0x08; //set third keyboard column low
	sei();

	keys = pressed;
	
}

/********************
LED operation
***********************/

void green_LED_on(){
	PORTC |= 0x04;
}

void green_LED_off(){
	PORTC &= 0xFB;
}

void green_LED_toggle(){
	PINC |= 0x04;
}

void yellow_LED_on(){
	PORTC |= 0x08;
}

void yellow_LED_off(){
	PORTC &= 0xF7;
}

void yellow_LED_toggle(){
	PINC |= 0x08;
}


void red_LED_on(){
	PORTC |= 0x02;
}

void red_LED_off(){
	PORTC &= 0xFC;
}

void red_LED_toggle(){
	PINC |= 0x02;
}


/** Prepares the input_coords_packet with the given coordinates */

void prepare_input_coords_packet() {
	U8 i;
	for (i = 0; i < 2; i++) {
		input_coords_packet.data[i * 2] = (U8)(input_coords[i] >> 8);
		TWBR = input_coords_packet.data[i * 2];
		input_coords_packet.data[i * 2 + 1] = (U8)(input_coords[i]);
		TWBR = input_coords_packet.data[i * 2 + 1];
	}
	async_request_packet.data[0] = EMPTY_REQUEST;
	while(!can_tx(6, &async_request_packet));
}

/********************
Callback function for handling the incoming init CAN message
***********************/

void incoming_init(CAN_packet* p, unsigned char mob){
	cli();
	if ((p->data[0] == 0x01) /*&& (init_status == no_init)*/) {
		init_status = init_1;
		while(!can_tx(2, &init_ack_async_packet));
	} else if (p->data[0] == 0x02 && init_status == init_1) {
		init_status = init_2;
		reset_string();
		add_string("Init OK!", LINE_1);
		prepare_rx(4, 0x0A, 0xFF, incoming_incremental_out_value);
		prepare_rx(7, 0x0D, 0xFF, incoming_operation_mode_changed);
		prepare_rx(9, 0x10, 0xFF, incoming_controller_message);
		//prepare_rx(7, 0x07, 0xFF, incoming_incremental_in_value);
		init_status = running;
		operation_state = previous_operation_state = init;	
	}	
	sei();
}

/********************
Callback function for handling the incoming incremental_out_value CAN message
***********************/

void incoming_incremental_out_value(CAN_packet* p, unsigned char mob){
	cli();

	volatile U16 val1 = 0;
	volatile U16 val2 = 0;

	val1 = (U16)p->data[2];
	val1 = val1 << 8;
	val1 += (U16)p->data[3];

	val2 = (U16)p->data[0];
	val2 = val2 << 8;
	val2 += (U16)p->data[1];
	if ( (operation_state != bluetooth) && !( (operation_state == controller) && ((controller_state == input) || (invgeo_error == 1)) )) {
		add_U16_string(val1, LINE_3 + 3);
		add_U16_string(val2, LINE_3 + 12);
	}

	sei();
}

/********************
Callback function for handling the incoming incremental_in_value CAN message
***********************/

void incoming_incremental_in_value(CAN_packet* p, unsigned char mob){
	cli();

	volatile U16 val1 = 0;
	volatile U16 val2 = 0;

	val1 = (U16)p->data[0];
	val1 = val1 << 8;
	val1 += (U16)p->data[1];

	val2 = (U16)p->data[2];
	val2 = val2 << 8;
	val2 += (U16)p->data[3];

	add_U16_string(val1, LINE_3 + 3);
	add_U16_string(val2, LINE_3 + 12);

	sei();
}

/********************
Callback function for handling the incoming incoming_operation_mode CAN message
***********************/

void incoming_operation_mode_changed(CAN_packet* p, unsigned char mob){
	switch (p->data[0]) {
		case PRIMITIVE_MODE_ACK:
			operation_state = primitive;
			break;
		case CONTROLLER_MODE_ACK:
			operation_state = controller;
			break;
		case CONFIG_MODE_ACK:
			operation_state = config;
			break;
		case BLUETOOTH_MODE_ACK:
			operation_state = bluetooth;
			break;
		default: break;
	}
}

/** Callback function for handling the incoming controller_message CAN message */

void incoming_controller_message(CAN_packet* p, unsigned char mob) {
	if (operation_state != controller) return;
	switch (p->data[0]) {
		case CONTROLLER_HOLD_MESSAGE:
			controller_state = hold;
			break;
		case INVGEO_OK_MESSAGE:
			controller_state = accepted;
			break;
		case INVGEO_ERROR_MESSAGE:
			invgeo_error = 1;
			controller_state = hold;
			break;
		case ON_TRAJECTORY_MESSAGE:
			controller_state = trajectory;
			break;
		default: break;
	}
}

/********************
Sends async_request CAN messages asking to reset the incremental counter
***********************/
void request_incremental_counter_reset() {

	async_request_packet.data[0] = 0x07;
	while (!can_tx(6, &async_request_packet));
}

