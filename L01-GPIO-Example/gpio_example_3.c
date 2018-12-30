
// Objective:  key '7' Toggle RGB-Blue  ; '8' Toggle RGB-Red ; '9' Toggle RGB-Green
//             Toggle bit using XOR binary operation with that bit
//			   display key-press on LCD with info..
// Line01: Blue - Off
// Line02: Green - Off
// Line03: Red - Off
// Line04: Key = x
// Nu_LB-002: L00_04_Keypad_7seg.c

// need to add ScanKey.c to the project
// from "C:\Nuvoton\BSP Library\NUC100SeriesBSP_CMSIS_v1.05.003\NuvotonPlatform_Keil\Src\NUC1xx-LB_002\ScanKey.c"

//
// Smpl_7seg_keypad
//
// Input:  3x3 keypad (input = 1~9 when key is pressed, =0 when key is not pressed
// Output: 7-segment LEDs
//
/*--------------------------------------------------------------------------*/
/*                                                                          */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.              */
/* edited: june 2014: dejwoot.kha@mail.kmutt.ac.th                          */
/* edited: dec 2018: TM														*/	
/*--------------------------------------------------------------------------*/

#include <stdio.h>																											 
#include "NUC1xx.h"
#include "Seven_Segment.h"
#include "scankey.h"
#include "Driver\DrvUART.h"
#include "Driver\DrvGPIO.h"
#include "DrvSYS.h"

#define DELAY300ms	300000 // The maximal delay time is 335000 us.

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
	int8_t number,press=0,display=0xFF;
	char lcd3_buffer[18];
	
	Initial_pannel();  //call initial pannel function
	clr_all_pannal();
	
	Init_RGB_LED();	
	OpenKeyPad();
	 
	while(1) {
		
	  number = Scankey();           // scan keypad to get a number (1~9)
		if (number != 0)
		{
			press=number;
		}	
		show_seven_segment(0,number); // display number on 7-segment LEDs
		DrvSYS_Delay(5000);           // delay time for keeping 7-segment display 
		if (number == 7) {
				display ^= 0x1;
				GPIOA->DOUT ^= 0x1000;				// RGB-Blue is DOUT[12]. So XOR with 0x1000
				DrvSYS_Delay(DELAY300ms);				
		} else if (number == 8) {
			  display ^= 0x4;
				GPIOA->DOUT ^= 0x4000;				// RGB-Red is DOUT[14]. So XOR with 0x4000
				DrvSYS_Delay(DELAY300ms);
		} else if (number == 9) {
			  display ^= 0x2;
			  GPIOA->DOUT ^= 0x2000;				// RGB-Green is DOUT[13]. So XOR with 0x2000
				DrvSYS_Delay(DELAY300ms);
		}	
		// Actually, you can mix it. For example, 
		// if you want to toggle RGB, so you need to XOR with 0x7000

        // Display on LCD Screen
		if ((display & 0x1) == 1) 
				print_lcd(0, "Blue - Off");
		else 
				print_lcd(0, "Blue - On  ");
		
		if ((display & 0x2) != 0)
				print_lcd(1, "Green - Off");
		else
		  	print_lcd(1, "Green - On ");
		
		if ((display & 0x4) != 0)
				print_lcd(2, "Red - Off");
		else 
				print_lcd(2, "Red - On ");

		
		sprintf(lcd3_buffer,"Key = ");
		sprintf(lcd3_buffer+6,"%d",press);
		print_lcd(3, lcd3_buffer);
		
		close_seven_segment();	      // turn off 7-segment LEDs								 
		}
	}
