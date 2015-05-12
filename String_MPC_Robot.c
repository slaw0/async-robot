/****************************************************************************************************/
/*****	Includes																				*****/
/****************************************************************************************************/


#include "String_MPC_Robot.h"

/****************************************************************************************************/
/*****	Function definitions																	*****/
/****************************************************************************************************/

void reset_string() {
	int i;
	for (i = 0; i < STRING_BUFFER_MAX_LENGTH; i++) __string_buffer[i] = ' ';
}

void reset_line_1() {
	int i;
	for (i = 0; i < LINE_2; i++) __string_buffer[i] = ' ';
}

void reset_line_2() {
	int i;
	for (i = LINE_2; i < LINE_3; i++) __string_buffer[i] = ' ';
}

void reset_line_3() {
	int i;
	for (i = LINE_3; i < LINE_4; i++) __string_buffer[i] = ' ';
}

void reset_line_4() {
	int i;
	for (i = LINE_4; i < STRING_BUFFER_MAX_LENGTH; i++) __string_buffer[i] = ' ';
}


U8 add_string(char* in, U8 pos){
	int i,j;
	for (i = 0; in[i] != 0; i++) {}
	if (pos + i > STRING_BUFFER_MAX_LENGTH) return 0;
	for (j = 0; j < i; j++) __string_buffer[pos + j] = in[j];
	return 1;
}


U8 add_U16_string(U16 val, U8 pos){
	int i;
	U16 str_values[5];
	
	if (pos + 5 > STRING_BUFFER_MAX_LENGTH) return 0;

	if (val) {
		str_values[0] = val/10000;
		val -= str_values[0] * 10000;
		str_values[1] = val/1000;
		val -= str_values[1] * 1000;
		str_values[2] = val/100;
		val -= str_values[2] * 100;
		str_values[3] = val/10;
		val -= str_values[3] * 10;
		str_values[4] = val;
		for (i = 0; str_values[i] == 0; i++) __string_buffer[pos + i] = ' ';
		for (; i < 5; i++) __string_buffer[pos + i]  = str_values[i] + 48;
	} else {
		for (i = 0; i < 4; i++) __string_buffer[pos + i] = ' ';
		__string_buffer[pos + i] = '0';
	}
	return 1;
}

