// step02 : key '1' reset WDT
// step03 : Button 'Int1' toggle WTIE
// Nu_LB-002: L01_03_Timer_toggle.c
// toggle every 1 second (50% duty cycle) -> GPB8

/*--------------------------------------------------------------------------*/
/*                                                                          */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.              */
/* edited: june 2014: dejwoot.kha@mail.kmutt.ac.th                          */
/*--------------------------------------------------------------------------*/

#include <stdio.h>	
#include <string.h>																										 
#include "NUC1xx.h"
#include "LCD_Driver.h"
#include "Driver\DrvGPIO.h"
#include "Driver\DrvSYS.h"
#include "lib_timer.h"

#define DELAY300ms	300000 		// The maximal delay time is 335000 us.

int8_t check = 0;
	
void EINT1Callback(void) {
	check ^= 1;
	if (check == 0) {
		UNLOCKREG();
		WDT->WTCR.WTIE = 0;
		LOCKREG();
	} else if (check == 1) {
		UNLOCKREG();
		WDT->WTCR.WTIE = 1;
		LOCKREG();
	}
	
	clr_all_pannal();			
	print_lcd(3,"                  ");
	print_lcd(0, "Int1 !!!!");
}

void InitWDT(void) {
	UNLOCKREG();		
	/* Step 1. Enable and Select WDT clock source */         
	SYSCLK->CLKSEL1.WDT_S = 3;	// Select 10kHz for WDT clock source  	
	SYSCLK->APBCLK.WDT_EN = 1;	// Enable WDT clock source	

	/* Step 2. Select Timeout Interval */
	WDT->WTCR.WTIS = 6;					// 2^16 * (1/10k) = 6.5536 sec.
	
	/* Step 3. Disable Watchdog Timer Reset function */
	WDT->WTCR.WTRE = 0;

	/* Step 4. Enable WDT interrupt */	
	WDT->WTCR.WTIF = 1;					// Write 1 to clear flag	
	WDT->WTCR.WTIE = 1;
	NVIC_EnableIRQ(WDT_IRQn);

	/* Step 5. Enable WDT module */
	WDT->WTCR.WTE = 1;					// Enable WDT
	WDT->WTCR.WTR = 1;					// Clear WDT counter
	LOCKREG();
	}
	
void WDT_IRQHandler(void) {
	UNLOCKREG();
	WDT->WTCR.WTIF = 1;
	WDT->WTCR.WTR = 1;
	LOCKREG();
	print_lcd(3, "WDT interrupt");
	}


int32_t main (void) {
	int8_t number;
	
	UNLOCKREG();
	SYSCLK->PWRCON.XTL12M_EN = 1;
	SYSCLK->CLKSEL0.HCLK_S = 0;
	LOCKREG();

	Initial_pannel();  //call initial pannel function
	clr_all_pannal();
	
	/* Configure general GPIO interrupt */
	DrvGPIO_Open(E_GPB, 15, E_IO_INPUT); // set pin 15 as an input

	/* Configure external interrupt */
    // If we want to create interrupt, connect the GP8 to GPB15
	DrvGPIO_EnableEINT1(E_IO_BOTH_EDGE, E_MODE_EDGE, EINT1Callback);

	OpenKeyPad();
	                        
	InitTIMER0();
  InitWDT();
	
	while(1) {
		number = Scankey();
		if (number==1)
		{
			UNLOCKREG();
			WDT->WTCR.WTR = 1;					// Clear WDT counter
			LOCKREG();
		}
	}
}
