#ifndef STRING_MPC_ROBOT_H
#define STRING_MPC_ROBOT_H

/****************************************************************************************************/
/*****	Includes  																				*****/
/****************************************************************************************************/

#include "include.h"
#include "assert.h"

/****************************************************************************************************/
/*****	Defines  																				*****/
/****************************************************************************************************/

#define STRING_BUFFER_MAX_LENGTH 80
#define LINE_1 0
#define LINE_2 20
#define LINE_3 40
#define LINE_4 60

/****************************************************************************************************/
/*****	Global variables used by strings														*****/
/****************************************************************************************************/

char __string_buffer[STRING_BUFFER_MAX_LENGTH];


/****************************************************************************************************/
/*****	Function prototypes																		*****/
/****************************************************************************************************/

U8 add_string(char* in, U8 pos);
U8 add_U16_string(U16 val, U8 pos);
void reset_string();
void reset_line_1();
void reset_line_2();
void reset_line_3();
void reset_line_4();

#endif //STRING_MPC_ROBOT_H
