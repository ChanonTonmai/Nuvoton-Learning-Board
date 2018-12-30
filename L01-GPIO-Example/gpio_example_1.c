
// Objective :   key '1' Turn On RGB-Blue ; '2' Turn On RGB-Green ;
//                   '3' Turn On RGB-Red  ; '4' Turn Off RGB-Blue ;
//					 '5' Turn off RGB-Green ; '6' Turn off RGB-Red ;
// Nu_LB-002: L00_04_Keypad_7seg.c

// need to add ScanKey.c to the project
// from "C:\Nuvoton\BSP Library\NUC100SeriesBSP_CMSIS_v1.05.003\NuvotonPlatform_Keil\Src\NUC1xx-LB_002\ScanKey.c"

// Smpl_7seg_keypad
//
// Input:  3x3 keypad (input = 1~9 when key is pressed, =0 when key is not pressed
// Output: 7-segment LEDs
//
/*--------------------------------------------------------------------------*/
/*                                                                          */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.              */
/* edited: june 2014: dejwoot.kha@mail.kmutt.ac.th                          */ 
/* edited: dec 2018: TM														*/													*/
/*--------------------------------------------------------------------------*/

#include <stdio.h>																											 
#include "NUC1xx.h"
#include "DrvSYS.h"
#include "Seven_Segment.h"
#include "scankey.h"
#include "Driver\DrvGPIO.h"

void Init_RGB_LED() {
	// initialize GPIO pins
	DrvGPIO_Open(E_GPA, 12, E_IO_OUTPUT); // GPA12 pin set to output mode
	DrvGPIO_Open(E_GPA, 13, E_IO_OUTPUT); // GPA13 pin set to output mode
	DrvGPIO_Open(E_GPA, 14, E_IO_OUTPUT); // GPA14 pin set to output mode
	// set GPIO pins output Hi to disable LEDs
	DrvGPIO_SetBit(E_GPA, 12); // GPA12 pin output Hi to turn off Blue  LED
	DrvGPIO_SetBit(E_GPA, 13); // GPA13 pin output Hi to turn off Green LED
	DrvGPIO_SetBit(E_GPA, 14); // GPA14 pin output Hi to turn off Red   LED
} 

int32_t main (void) {
	int8_t number;
	
	Init_RGB_LED();	
	OpenKeyPad();					 	
	 
	while(1) {
	  number = Scankey();           // scan keypad to get a number (1~9)
		show_seven_segment(0,number); // display number on 7-segment LEDs
		DrvSYS_Delay(5000);           // delay time for keeping 7-segment display 
		
		if (number == 1) {
				DrvGPIO_ClrBit(E_GPA,12);  // GPA12 = Blue,  0 : on, 1 : off
		} else if (number == 2) {
				DrvGPIO_ClrBit(E_GPA,13);  // GPA13 = Green,  0 : on, 1 : off
		} else if (number == 3) {
				DrvGPIO_ClrBit(E_GPA,14);  // GPA14 = Red,  0 : on, 1 : off
		} else if (number == 4){
				DrvGPIO_SetBit(E_GPA,12);  // GPA12 = Blue,  0 : on, 1 : off
		} else if (number == 5) { 
			  DrvGPIO_SetBit(E_GPA,13);  // GPA13 = Green,  0 : on, 1 : off
		} else if (number == 6) { 
				DrvGPIO_SetBit(E_GPA,14);  // GPA14 = Red,  0 : on, 1 : off
		}
		close_seven_segment();	      // turn off 7-segment LEDs			
					
	}
}
