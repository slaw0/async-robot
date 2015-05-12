/****************************************************************************************************/
/*																									*/
/* INCLUDE.H																						*/
/*																									*/
/* Author:	Alexandros Soumelidis																	*/
/* Date:	03 August 2006																			*/
/*																									*/
/****************************************************************************************************/
/*																									*/
/* Header file for AVRstamp basic application														*/
/*																									*/
/****************************************************************************************************/
/* Compiler: 		AVR-GCC																			*/
/* Target platform: AVRstamp1.3FUSB-CAN																*/
/****************************************************************************************************/

#ifndef INCLUDE_H
#define INCLUDE_H

/****************************************************************************************************/
/***** Common data types																		*****/
/****************************************************************************************************/

typedef unsigned char		BOOL;

typedef unsigned char		BYTE;
typedef unsigned short		WORD;
typedef unsigned long		DWORD;
typedef unsigned long long	QWORD;

typedef unsigned char		UINT8;
typedef unsigned short		UINT16;
typedef unsigned long		UINT32;
typedef unsigned long long	UINT64;

typedef signed char			INT8;
typedef signed short		INT16;
typedef signed long			INT32;
typedef signed long long	INT64;



typedef unsigned char      Bool;

typedef unsigned char       U8 ;
typedef unsigned short      U16;
typedef unsigned long       U32;
typedef unsigned long long  U64;
typedef signed char         S8 ;
typedef signed short        S16;
typedef signed long         S32;
typedef signed long long    S64;

typedef union
{
  U16 h   ;     // h as HALF-WORD
  U8  b[2];     // b as BYTE
} Union16;

typedef union
{
  U32 w   ;     // w as WORD
  U16 h[2];     // h as HALF-WORD
  U8  b[4];     // b as BYTE
} Union32;

typedef union
{
  U64 d   ;     // d as DOUBLE-WORD
  U32 w[2];     // w as WORD
  U16 h[4];     // h as HALF-WORD
  U8  b[8];     // b as BYTE
} Union64;

/****************************************************************************************************/
/***** Common constant definitions																*****/
/****************************************************************************************************/

#ifndef FALSE
	#define FALSE 0
#endif
#ifndef TRUE
	#define TRUE 1
#endif
#ifndef NULL
	#define NULL 0
#endif

enum { UP, DOWN };
enum { FORWARD, REVERSE };

//! Constants
#define ENABLE   1
#define ENABLED  1
#define DISABLED 0
#define DISABLE  0

#define KO      0
#define OK      1
#define CLR     0
#define SET     1
#define OFF     0
#define ON      1

/****************************************************************************************************/

/****************************************************************************************************/
/***** Macro definitions definitions															*****/
/****************************************************************************************************/

#define BM(n) (1 << (n))
#define BF(x,b,s) (((x) & (b)) >> (s))
#define MIN(n,m) (((n) < (m)) ? (n) : (m))
#define MAX(n,m) (((n) < (m)) ? (m) : (n))
#define ABS(n) ((n < 0) ? -(n) : (n))
#define NOP() asm volatile ("nop\n\t" ::)

#define Max(a, b)          ( (a)>(b) ? (a) : (b) )
#define Min(a, b)          ( (a)<(b) ? (a) : (b) )
#define Align_up(val, n)   ( ((val)+(n)-1) & ~((n)-1) )
#define Align_down(val, n) (  (val)        & ~((n)-1) )
//! Bit and bytes manipulations
#define Low(data_w)                ((U8)data_w)
#define High(data_w)               ((U8)(data_w>>8))
#define Tst_bit_x(addrx,mask)   (*addrx & mask)
#define Set_bit_x(addrx,mask)   (*addrx = (*addrx |  mask))
#define Clr_bit_x(addrx,mask)   (*addrx = (*addrx & ~mask))



/****************************************************************************************************/
/*****	Standard GCC include files for AVR														*****/
/****************************************************************************************************/

//#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

/****************************************************************************************************/
/*****	Function prototypes																		*****/
/****************************************************************************************************/

/****************************************************************************************************/
/*																										*/
/*	Function:		void halWaitX10 (UINT16 timeout)													*/
/*																										*/
/*	Destriction:	Runs an idle loop for [timeout] 10 microseconds.									*/
/*																										*/
/*  Arguments:		UINT16 timeout		the timeout in 10 microseconds units							*/
/*																										*/
/****************************************************************************************************/

void halWait(UINT16 timeout);

/****************************************************************************************************/
/*																										*/
/*	Function:		void halWaitX10 (UINT16 timeout)													*/
/*																										*/
/*	Destriction:	Runs an idle loop for [timeout] 10 microseconds.									*/
/*																										*/
/*  Arguments:		UINT16 timeout		the timeout in 10 microseconds units							*/
/*																										*/
/****************************************************************************************************/

void halWaitX10 (UINT16 timeout);

/****************************************************************************************************/

#endif

/****************************************************************************************************/
