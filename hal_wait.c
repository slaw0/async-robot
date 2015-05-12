/********************************************************************************************************/
/*                                                                                                      */
/*        **********                                                                                    */
/*                                                                                                      */
/********************************************************************************************************/
/*																										*/
/********************************************************************************************************/
/* Compiler: AVR-GCC																					*/
/* Target platform: AVRstamp1.2Fusb																		*/
/********************************************************************************************************/
/*																										*/
/********************************************************************************************************/

#include "include.h"
#include "hal.h"

/********************************************************************************************************/
/*																										*/
/*	Function:		void halWait (UINT16 timeout)														*/
/*																										*/
/*	Destriction:	Runs an idle loop for [timeout] microseconds.										*/
/*																										*/
/*  Arguments:		UINT16 timeout					the timeout in microseconds							*/
/*																										*/
/********************************************************************************************************/

void halWait (UINT16 timeout)
{
    // Time base: 14,745,600 cycles per sec: 15 cycles; error 1.75% 
    do {
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
	   } while (--timeout);

} // halWait

/********************************************************************************************************/
/*																										*/
/*	Function:		void halWaitX10 (UINT16 timeout)													*/
/*																										*/
/*	Destriction:	Runs an idle loop for [timeout] 10 microseconds.									*/
/*																										*/
/*  Arguments:		UINT16 timeout					the timeout in 10 microseconds units				*/
/*																										*/
/********************************************************************************************************/

void halWaitX10 (UINT16 timeout)
{
    // Time base: 14,745,600 cycles per sec: 147 cycles; error 0.3%
    do {
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();

		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
		 NOP();
	   } while (--timeout);

} // halWaitX10

/********************************************************************************************************/
