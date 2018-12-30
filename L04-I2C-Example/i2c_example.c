// step01 : key '5','6' shift in from the right
//			   '7','8' shift in from the left
//			   '9' count up 00-FF
// step02 : add capture6 (GE0)
// Nu_LB-002: L03_02_I2C_8574_i2c1_displayDIG.c
/*	
	key '1'-'4'
	key '3' count up 

	connected to 8574
	GPA10 - SDA
	GPA11 - SCL
	
	addr 0x70 the right 7SEG
	addr 0x72 the left 7SEG
*/

/*--------------------------------------------------------------------------*/
/*                                                                          */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.              */
/* edited: june 2014: dejwoot.kha@mail.kmutt.ac.th                          */
/*--------------------------------------------------------------------------*/

#include <stdio.h>
#include "NUC1xx.h"
#include "Driver\DrvSYS.h"
#include "Driver\DrvGPIO.h"
#include "LCD_Driver.h"
#include "EEPROM_24LC64.h"
#include "Driver\DrvI2C.h"
#include "scankey.h"

#define DELAY300ms	300000 // The maximal delay time is 335000 us.
#define	PWM_CNR	0xFFFF

static uint16_t Timer3Counter=0;
uint16_t	CaptureCounter6 = 0;
uint32_t	CaptureValue6[2];
//------------------------------------------------------------------Capture6
void InitCapture6(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPEMFP.PWM6 = 1;
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM67_EN = 1; // Enable PWM clock
	SYSCLK->CLKSEL2.PWM67_S = 0; // Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz
	
	PWMB->PPR.CP23 = 11;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMB->CSR.CSR2 = 4;			// clock divider -> 0:/2, 1:/4, 2:/8, 3:/16, 4:/1
									         
	/* Step 3. Select PWM Operation mode */
	PWMB->PCR.CH2MOD = 1;		// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD from 0 to 1.
	PWMB->CNR2 = PWM_CNR;			// Set Reload register
	PWMB->CAPENR = 4;				// Enable Capture function pin
	PWMB->CCR2.CAPCH2EN = 1;// Enable Capture function

	/* Step 4. Set PWM Interrupt */
	PWMB->CCR2.CRL_IE2 = 1;	// Enable Capture rising edge interrupt
	PWMB->CCR2.CFL_IE2 = 1;	// Enable Capture falling edge interrupt
	PWMB->PIER.PWMIE2 = 1;	// Enable PWM interrupt for down-counter equal zero.
	NVIC_EnableIRQ(PWMB_IRQn);  // Enable PWM inturrupt

	/* Step 5. Enable PWM down counter*/
	PWMB->PCR.CH2EN = 1;		// Enable PWM down counter
	}
uint8_t HEX2Disp(uint8_t hexNum) {	
	static const uint8_t lookUp[16] = {
		0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8,
		0x80, 0x90, 0x88, 0x83, 0xC6, 0xA1, 0x86, 0x8E
		};
	uint8_t hexDisp = lookUp[hexNum];
	return hexDisp;
	}
	
void Write_to_any8574(uint8_t i2c_addr, uint8_t data) {
	uint32_t i;
	SystemCoreClock = DrvSYS_GetHCLKFreq();
	//Open I2C1 and set clock = 50Kbps
	DrvI2C_Open(I2C_PORT1, 50000);
	
	//send i2c start
	DrvI2C_Ctrl(I2C_PORT1, 1, 0, 0, 0);	// set start
	while (I2C1->I2CON.SI == 0);				// poll si flag
	
	//send writer command
	I2C1->I2CDAT = i2c_addr;						// send writer command to 8574
	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 0); // clr si flag
	while (I2C1->I2CON.SI == 0);				// poll si flag

	//send data
	I2C1->I2CDAT = data;								// write data to 
	DrvI2C_Ctrl(I2C_PORT1, 0, 0, 1, 1); // clr si and set ack	
	while (I2C1->I2CON.SI == 0);				// poll si flag
   	
	//send i2c stop
	DrvI2C_Ctrl(I2C_PORT1, 0, 1, 1, 0); // send stop	
	while (I2C1->I2CON.STO);						/* if a STOP condition is detected 
										this flag will be cleared by hardware automatically. */
	//while (I2C1->I2CON.SI == 0);			// poll si flag
	
	for(i=0;i<60;i++);
	DrvI2C_Close(I2C_PORT1);
	for(i=0;i<6000;i++);
	for(i=0;i<6000;i++);
	}

//----------------------------------------------------------------------PWM1
void InitPWM1(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPAMFP.PWM1_AD14 = 1;		// System Manager Control Registers
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM01_EN = 1;	// Enable PWM clock
	SYSCLK->CLKSEL1.PWM01_S = 0;	// Select 12MHz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz 
	PWMA->PPR.CP01 = 11;	// Prescaler 0~255, Setting 0 to stop output clock
	PWMA->CSR.CSR1 = 3;		// clock divider ->	0:/2, 1:/4, 2:/8, 3:/16, 4:/1
	// PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]
	// Ex:= 12M/[(11+1)*(16)*(2^16)] = 0.95367 Hz -> T = 1.048576 ~ 262+786

	/* Step 3. Select PWM Operation mode */
	PWMA->PCR.CH1MOD = 1;		// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD form 0 to 1.
	PWMA->CNR1 = 0xFFFF;
	PWMA->CMR1 = 0x3FFF;
	// CMR < CNR:	PWM low width = (CNR-CMR) unit [one PWM clock cycle]
	//						PWM high width = (CMR+1) unit 

	PWMA->PCR.CH1INV = 0;	// Inverter -> 0:off, 1:on
	PWMA->PCR.CH1EN = 1;	// PWM function -> 0:Disable, 1:Enable
 	PWMA->POE.PWM1 = 1;		// Output to pin -> 0:Diasble, 1:Enable
	}
//----------------------------------------------------------------------PWM2
void InitPWM2(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPAMFP.PWM2_AD15 = 1;		// System Manager Control Registers
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM23_EN = 1;	// Enable PWM clock
	SYSCLK->CLKSEL1.PWM23_S = 0;	// Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz 
	PWMA->PPR.CP23 = 1;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMA->CSR.CSR2 = 4;		// clock divider ->	0:/2, 1:/4, 2:/8, 3:/16, 4:/1
	// PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]
	// Ex:= 12M/[(1+1)*(1)*(8191)] = 732.5 Hz -> T = 1365 micros. ~ 682+682
	
	/* Step 3. Select PWM Operation mode */
	PWMA->PCR.CH2MOD = 1;	// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD form 0 to 1.
	PWMA->CNR2 = 0x1FFF;	// 0x1FFF = 8191
	PWMA->CMR2 = 0x0FFF;

	PWMA->PCR.CH2INV = 0;	//Inverter->0:off, 1:on
	PWMA->PCR.CH2EN = 1;	//PWM function->0:Disable, 1:Enable
 	PWMA->POE.PWM2 = 1;		//Output to pin->0:Diasble, 1:Enable
	}
//------------------------------------------------------------------PWMA_IRQ
void PWMB_IRQHandler(void) {		// PWM interrupt subroutine 
	if (PWMB->PIIR.PWMIF2) {
		CaptureCounter6++;						// Delay (PWM_CNR+1) usec
		if (CaptureCounter6 == 0) {	// Overflow
			}
		PWMB->PIIR.PWMIF2	=	1;			// write 1 to clear this bit to zero
		}
	if (PWMB->CCR2.CAPIF2) {
		if (PWMB->CCR2.CFLRI2) {		// Calculate High Level width
			CaptureValue6[0] = CaptureCounter6*(PWM_CNR+1)+(PWM_CNR-PWMB->CFLR2);//usec
			CaptureCounter6 = 0;				// reset
			PWMB->CCR2.CFLRI2 = 0;// write 0 to clear this bit to zero if BCn bit is 0
		}
		if (PWMB->CCR2.CRLRI2) {		//Calculate Low Level width
			CaptureValue6[1] = CaptureCounter6*(PWM_CNR+1)+(PWM_CNR-PWMB->CRLR2);//usec
			CaptureCounter6 = 0;				// reset
			PWMB->CCR2.CRLRI2 = 0;// write 0 to clear this bit to zero if BCn bit is 0	
			}
		PWMB->CCR2.CAPIF2 = 1;	// write 1 to clear this bit to zero
		}
	}
//--------------------------------------------------------------------Timer3
void InitTIMER3(void) {
	/* Step 1. Enable and Select Timer clock source */          
	SYSCLK->CLKSEL1.TMR3_S = 0;	// Select 12Mhz for Timer3 clock source
	// 0 = 12 MHz, 1 = 32 kHz, 2 = HCLK, 7 = 22.1184 MHz
	SYSCLK->APBCLK.TMR3_EN = 1;	// Enable Timer0 clock source

	/* Step 2. Select Operation mode */	
	TIMER3->TCSR.MODE = 1;			// 1 -> Select periodic mode
	// 0 = One shot, 1 = Periodic, 2 = Toggle, 3 = continuous counting mode
	
	/* Step 3. Select Time out period 
	= (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
	TIMER3->TCSR.PRESCALE = 11;	// Set Prescale [0~255]
	TIMER3->TCMPR = 1000000;		// Set TCMPR [0~16777215]
	// (1/12000000)*(11+1)*(1000000)= 1 sec or 1 Hz

	/* Step 4. Enable interrupt */
	TIMER3->TCSR.IE = 1;
	TIMER3->TISR.TIF = 1;				// Write 1 to clear the interrupt flag 		
	NVIC_EnableIRQ(TMR3_IRQn);	// Enable Timer0 Interrupt

	/* Step 5. Enable Timer module */
	TIMER3->TCSR.CRST = 1;			// Reset up counter
	TIMER3->TCSR.CEN = 1;				// Enable Timer0
	}
//----------------------------------------------------------------Timer3_IRQ
void TMR3_IRQHandler(void) {	// Timer0 interrupt subroutine
	char lcd1_buffer[18] = "Timer3:";
	char lcd2_buffer[18] = "High:";
	char lcd3_buffer[18] = "Low: ";
	
	Timer3Counter += 1;
	sprintf(lcd1_buffer+7, " %d s.", Timer3Counter);
	print_lcd(1, lcd1_buffer);
 	
	/* Display capture values */
	if (CaptureValue6[0] >= 1000000) {
		sprintf(lcd2_buffer+5, "%dsec", CaptureValue6[0]/1000000);
		} else if (CaptureValue6[0] >= 1000) {
		sprintf(lcd2_buffer+5, "%dmsec", CaptureValue6[0]/1000);
		} else
		sprintf(lcd2_buffer+5, "%dusec", CaptureValue6[0]);
	print_lcd(2, lcd2_buffer);

	if (CaptureValue6[1] >= 1000000) {
		sprintf(lcd3_buffer+5, "%dsec", CaptureValue6[1]/1000000);
		} else if (CaptureValue6[1] >= 1000) {
		sprintf(lcd3_buffer+5, "%dmsec", CaptureValue6[1]/1000);
		} else
		sprintf(lcd3_buffer+5, "%dusec", CaptureValue6[1]);
	print_lcd(3, lcd3_buffer);
	
	TIMER3->TISR.TIF = 1;    		// Write 1 to clear the interrupt flag 
	}

int main(void) {
	unsigned char temp, i ,keyin=0;

	/* Unlock the protected registers */	
	UNLOCKREG();
  /* Enable the 12MHz oscillator oscillation */
	DrvSYS_SetOscCtrl(E_SYS_XTL12M, 1);
	/* Waiting for 12M Xtal to stable */
	SysTimerDelay(5000);
	/* HCLK clock source. 0: external 12MHz; 4:internal 22MHz RC oscillator */
	DrvSYS_SelectHCLKSource(0);
	/*lock the protected registers */
	LOCKREG();				
	/* HCLK clock frequency = HCLK clock source / (HCLK_N + 1) */
	DrvSYS_SetClockDivider(E_SYS_HCLK_DIV, 0); 

	Initial_pannel();  // call initial panel function
	clr_all_pannal();
	
	InitPWM1();
	InitPWM2();
	InitCapture6(); //E0-A13/E0-A14
	InitTIMER3();
	
	print_lcd(0, "I2C with       ");
	print_lcd(1, "       24LC65  ");
	print_lcd(2, "       PCF8574 ");	  
	print_lcd(3, "press key1-key4");
	
	// initial keyboard
	for(i=0;i<6;i++)		
		DrvGPIO_Open(E_GPA, i, E_IO_QUASI);

	DrvGPIO_InitFunction(E_FUNC_I2C1);

	while(1) {
	  temp=Scankey();
		if (temp == 1 ) {
			print_lcd(0, "Key1 had pressed ");
			Write_to_any8574(0x70, 1);
			}
		if (temp == 2) {
			print_lcd(0, "Key2 had pressed ");
			Write_to_any8574(0x70, 0xFF);
			}
		if (temp == 3) {
			print_lcd(0, "Key3 had pressed ");

			for(i=0;i<16;i++) {
				Write_to_any8574(0x72, HEX2Disp(i));
				DrvSYS_Delay(DELAY300ms);	   	// delay
				}
			}
		if (temp == 4) {
			print_lcd(0, "Key4 had pressed ");
			Write_to_any8574(0x72, 0xFF);
			}
		if (temp == 5) {
			print_lcd(0, "Key5 had pressed ");
			keyin = keyin << 4;
			keyin = keyin | temp;
			Write_to_any8574(0x70, HEX2Disp(keyin&0x0F));
			Write_to_any8574(0x72, HEX2Disp(keyin>>4));
			DrvSYS_Delay(DELAY300ms);	   	// delay
			}
		if (temp == 6) {
			print_lcd(0, "Key6 had pressed ");
			keyin = keyin << 4;
			keyin = keyin | temp;
			Write_to_any8574(0x70, HEX2Disp(keyin&0x0F));
			Write_to_any8574(0x72, HEX2Disp(keyin>>4));
			DrvSYS_Delay(DELAY300ms);	   	// delay
			}
		if (temp == 7) {
			print_lcd(0, "Key7 had pressed ");
			keyin = keyin >> 4;
			temp = temp << 4;
			keyin = keyin | temp;
			Write_to_any8574(0x70, HEX2Disp(keyin&0x0F));
			Write_to_any8574(0x72, HEX2Disp(keyin>>4));
			DrvSYS_Delay(DELAY300ms);	   	// delay
			}
		if (temp == 8) {
			print_lcd(0, "Key8 had pressed ");
			keyin = keyin >> 4;
			temp = temp << 4;
			keyin = keyin | temp;
			Write_to_any8574(0x70, HEX2Disp(keyin&0x0F));
			Write_to_any8574(0x72, HEX2Disp(keyin>>4));
			DrvSYS_Delay(DELAY300ms);	   	// delay
			}
		if (temp == 9) {
			print_lcd(0, "Key9 had pressed ");

			for(i=0;i<256;i++) {
				Write_to_any8574(0x70, HEX2Disp(i&0x0F));
			    Write_to_any8574(0x72, HEX2Disp(i>>4));
				DrvSYS_Delay(200000);	   	// delay
				}
			}
		}
}

