/*--------------------------------------------------------------------------*/
/*                                                                          */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.              */
/* edited: dec 2018: chanon.khong@mail.kmutt.ac.th                          */
/*--------------------------------------------------------------------------*/

#include <stdio.h>	
#include <string.h>																										 
#include "NUC1xx.h"
#include "LCD_Driver.h"
#include "lib_timer.h"

static uint16_t Timer0Counter=0;
static uint16_t Timer1Counter=0;
static uint16_t Timer2Counter=0;
static uint16_t Timer3Counter=0;

void InitTIMER0(void) {
	/* Step 0. GPIO initial */
	SYS->GPBMFP.TM0 = 1;				// System Manager Control Registers
	
	/* Step 1. Enable and Select Timer clock source */          
	SYSCLK->CLKSEL1.TMR0_S = 0;	// Select 12Mhz for Timer0 clock source
	// 0 = 12 MHz, 1 = 32 kHz, 2 = HCLK, 7 = 22.1184 MHz
	SYSCLK->APBCLK.TMR0_EN = 1;	// Enable Timer0 clock source

	/* Step 2. Select Operation mode */	
	TIMER0->TCSR.MODE = 2;			// 2 -> Select Toggle mode
	// 0 = One shot, 1 = Periodic, 2 = Toggle, 3 = continuous counting mode
	
	/* Step 3. Select Time out period 
	= (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
	TIMER0->TCSR.PRESCALE = 11;	// Set Prescale [0~255]
	TIMER0->TCMPR = 1000000;		// Set TCMPR [0~16777215]
	// (1/12000000)*(11+1)*(1000000)= 1 sec or 1 Hz

	/* Step 4. Enable interrupt */
	TIMER0->TCSR.IE = 1;
	TIMER0->TISR.TIF = 1;				// Write 1 to clear the interrupt flag 		
	NVIC_EnableIRQ(TMR0_IRQn);	// Enable Timer0 Interrupt

	/* Step 5. Enable Timer module */
	TIMER0->TCSR.CRST = 1;			// Reset up counter
	TIMER0->TCSR.CEN = 1;				// Enable Timer0
}

void TMR0_IRQHandler(void) {	// Timer0 interrupt subroutine
	
	char lcd2_buffer[18] = "Timer0:";
	Timer0Counter += 1;
	sprintf(lcd2_buffer+7, " %d s.", Timer0Counter);
	print_lcd(2, lcd2_buffer);
	TIMER0->TISR.TIF = 1;    		// Write 1 to clear the interrupt flag 
	}

void InitTIMER1(void) {
	/* Step 0. GPIO initial */
	SYS->GPBMFP.TM1_SS11 = 1;				// System Manager Control Registers
	
	/* Step 1. Enable and Select Timer clock source */          
	SYSCLK->CLKSEL1.TMR1_S = 0;	// Select 12Mhz for Timer0 clock source
	// 0 = 12 MHz, 1 = 32 kHz, 2 = HCLK, 7 = 22.1184 MHz
	SYSCLK->APBCLK.TMR1_EN = 1;	// Enable Timer1 clock source

	/* Step 2. Select Operation mode */	
	TIMER1->TCSR.MODE = 2;			// 2 -> Select Toggle mode
	// 0 = One shot, 1 = Periodic, 2 = Toggle, 3 = continuous counting mode
	
	/* Step 3. Select Time out period 
	= (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
	TIMER1->TCSR.PRESCALE = 11;	// Set Prescale [0~255]
	TIMER1->TCMPR = 1000000;		// Set TCMPR [0~16777215]
	// (1/12000000)*(11+1)*(1000000)= 1 sec or 1 Hz

	/* Step 4. Enable interrupt */
	TIMER1->TCSR.IE = 1;
	TIMER1->TISR.TIF = 1;				// Write 1 to clear the interrupt flag 		
	NVIC_EnableIRQ(TMR1_IRQn);	// Enable Timer1 Interrupt

	/* Step 5. Enable Timer module */
	TIMER1->TCSR.CRST = 1;			// Reset up counter
	TIMER1->TCSR.CEN = 1;				// Enable Timer0
}

void TMR1_IRQHandler(void) {	// Timer1 interrupt subroutine
	
	char lcd2_buffer[18] = "Timer1:";
	Timer1Counter += 1;
	sprintf(lcd2_buffer+7, " %d s.", Timer1Counter);
	print_lcd(2, lcd2_buffer);
	TIMER1->TISR.TIF = 1;    		// Write 1 to clear the interrupt flag 
	}

void InitTIMER2(void) {
	/* Step 0. GPIO initial */
	SYS->GPBMFP.TM2_SS01 = 1;				// System Manager Control Registers
	
	/* Step 1. Enable and Select Timer clock source */          
	SYSCLK->CLKSEL1.TMR2_S = 0;	// Select 12Mhz for Timer2 clock source
	// 0 = 12 MHz, 1 = 32 kHz, 2 = HCLK, 7 = 22.1184 MHz
	SYSCLK->APBCLK.TMR2_EN = 1;	// Enable Timer1 clock source

	/* Step 2. Select Operation mode */	
	TIMER2->TCSR.MODE = 2;			// 2 -> Select Toggle mode
	// 0 = One shot, 1 = Periodic, 2 = Toggle, 3 = continuous counting mode
	
	/* Step 3. Select Time out period 
	= (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
	TIMER2->TCSR.PRESCALE = 11;	// Set Prescale [0~255]
	TIMER2->TCMPR = 1000000;		// Set TCMPR [0~16777215]
	// (1/12000000)*(11+1)*(1000000)= 1 sec or 1 Hz

	/* Step 4. Enable interrupt */
	TIMER2->TCSR.IE = 1;
	TIMER2->TISR.TIF = 1;				// Write 1 to clear the interrupt flag 		
	NVIC_EnableIRQ(TMR2_IRQn);	// Enable Timer1 Interrupt

	/* Step 5. Enable Timer module */
	TIMER2->TCSR.CRST = 1;			// Reset up counter
	TIMER2->TCSR.CEN = 1;				// Enable Timer0
}

void TMR2_IRQHandler(void) {	// Timer2 interrupt subroutine
	
	char lcd2_buffer[18] = "Timer2:";
	Timer2Counter += 1;
	sprintf(lcd2_buffer+7, " %d s.", Timer2Counter);
	print_lcd(2, lcd2_buffer);
	TIMER2->TISR.TIF = 1;    		// Write 1 to clear the interrupt flag 
	}

void InitTIMER3(void) {
	/* Step 0. GPIO initial */
	SYS->GPBMFP.TM3_PWM4 = 1;				// System Manager Control Registers
	
	/* Step 1. Enable and Select Timer clock source */          
	SYSCLK->CLKSEL1.TMR3_S = 0;	// Select 12Mhz for Timer0 clock source
	// 0 = 12 MHz, 1 = 32 kHz, 2 = HCLK, 7 = 22.1184 MHz
	SYSCLK->APBCLK.TMR3_EN = 1;	// Enable Timer0 clock source

	/* Step 2. Select Operation mode */	
	TIMER3->TCSR.MODE = 2;			// 2 -> Select Toggle mode
	// 0 = One shot, 1 = Periodic, 2 = Toggle, 3 = continuous counting mode
	
	/* Step 3. Select Time out period 
	= (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
	TIMER3->TCSR.PRESCALE = 11;	// Set Prescale [0~255]
	TIMER3->TCMPR = 1000000;		// Set TCMPR [0~16777215]
	// (1/12000000)*(11+1)*(1000000)= 1 sec or 1 Hz

	/* Step 4. Enable interrupt */
	TIMER3->TCSR.IE = 1;
	TIMER3->TISR.TIF = 1;				// Write 1 to clear the interrupt flag 		
	NVIC_EnableIRQ(TMR3_IRQn);	// Enable TIMER3 Interrupt

	/* Step 5. Enable Timer module */
	TIMER3->TCSR.CRST = 1;			// Reset up counter
	TIMER3->TCSR.CEN = 1;				// Enable TIMER3
	}
	
void TMR3_IRQHandler(void) {	// Timer3 interrupt subroutine
	char lcd2_buffer[18] = "Timer3:";
	Timer3Counter += 1;
	sprintf(lcd2_buffer+7, " %d s.", Timer3Counter);
	print_lcd(2, lcd2_buffer);
	TIMER3->TISR.TIF = 1;    		// Write 1 to clear the interrupt flag 
	}