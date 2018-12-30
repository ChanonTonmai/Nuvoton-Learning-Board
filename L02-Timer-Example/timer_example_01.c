// Objective : key '1' reset WDT
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
#include "lib_timer.h" // timer library that have timer0->timer3

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

	OpenKeyPad();
	                        
	InitTIMER2();
    InitWDT();
	
	while(1) {
		number = Scankey();
		if (number == 1)
		{
			UNLOCKREG();
			WDT->WTCR.WTR = 1;					// Clear WDT counter
			LOCKREG();
		}
	}
}
