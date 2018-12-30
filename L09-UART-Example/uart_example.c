//step01: UART0 Tx(B1) -> UART2 Rx(D14)
//stepA:  Capture3 with PWM1,PWM2 turn on LED6,7\
//step03: button 'EINT1' Toggle LED'On'
//				5->6->7->8 then 'Off' 8->7->6->5
// Nu_LB-002: L08_01_UART0_keyPad.c
/* 
connect GPB0 to GPB1

UART0:GPB0 - Rx
			GPB1 - Tx
UART1:GPB4 - Rx
			GPB5 - Tx
UART2:GPD14 - Rx
			GPD15 - Tx			
*/

/*--------------------------------------------------------------------------*/
/*                                                                          */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.              */
/* edited: june 2014: dejwoot.kha@mail.kmutt.ac.th                          */
/*--------------------------------------------------------------------------*/

#include <stdio.h>
#include "Driver\DrvUART.h"
#include "Driver\DrvGPIO.h"
#include "Driver\DrvSYS.h"
#include "NUC1xx.h"
#include "NUC1xx-LB_002\LCD_Driver.h"
#include "scankey.h"

#define	PWM_CNR	0xFFFF
#define DELAY300ms	300000 // The maximal delay time is 335000 us.

static uint16_t Timer3Counter=0;
uint16_t	CaptureCounter = 0;
uint32_t	CaptureValue[2];
uint16_t toggle=1;

volatile uint8_t comRbuf[16];
volatile uint16_t comRbytes = 0;

char TEXT2[16] = "RX:             ";

//-------------------------------------------------------------UART_Callback
void UART0_INT_HANDLE(void) {
	uint8_t bInChar[1] = {0xFF};

	while (UART0->ISR.RDA_IF == 1) {	// Receive Data Available Interrupt Flag
		DrvUART_Read(UART_PORT2,bInChar, 1);	
		if (comRbytes < 2) { // check if Buffer is full
			comRbuf[comRbytes] = bInChar[0];
			comRbytes++;
			}
		else if (comRbytes == 2) {
			comRbuf[comRbytes] = bInChar[0];
			comRbytes = 0;
			sprintf(TEXT2+4, "%s", comRbuf);
			print_lcd(2, TEXT2);
			}
		}
	}
	
	//-------------------------------------------------------------UART_Callback
void UART2_INT_HANDLE(void) {
	uint8_t bInChar[1] = {0xFF};

	while (UART2->ISR.RDA_IF == 1) {	// Receive Data Available Interrupt Flag
		DrvUART_Read(UART_PORT2,bInChar, 1);	
		if (comRbytes < 2) { // check if Buffer is full
			comRbuf[comRbytes] = bInChar[0];
			comRbytes++;
			}
		else if (comRbytes == 2) {
			comRbuf[comRbytes] = bInChar[0];
			comRbytes = 0;
			sprintf(TEXT2+4, "%s", comRbuf);
			print_lcd(1, TEXT2);
			}
		}
	}
//------------------------------------------------------------------Capture0
void InitCapture3(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPAMFP.PWM3_I2SMCLK = 1;
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM23_EN = 1; // Enable PWM clock
	SYSCLK->CLKSEL1.PWM23_S = 0; // Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz
	
	PWMA->PPR.CP23 = 11;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMA->CSR.CSR3 = 4;			// clock divider -> 0:/2, 1:/4, 2:/8, 3:/16, 4:/1
									         
	/* Step 3. Select PWM Operation mode */
	PWMA->PCR.CH3MOD = 1;		// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD from 0 to 1.
	PWMA->CNR3 = PWM_CNR;			// Set Reload register
	PWMA->CAPENR = 8;				// Enable Capture function pin
	PWMA->CCR2.CAPCH3EN = 1;// Enable Capture function

	/* Step 4. Set PWM Interrupt */
	PWMA->CCR2.CRL_IE3 = 1;	// Enable Capture rising edge interrupt
	PWMA->CCR2.CFL_IE3 = 1;	// Enable Capture falling edge interrupt
	PWMA->PIER.PWMIE3 = 1;	// Enable PWM interrupt for down-counter equal zero.
	NVIC_EnableIRQ(PWMA_IRQn);  // Enable PWM inturrupt

	/* Step 5. Enable PWM down counter*/
	PWMA->PCR.CH3EN = 1;		// Enable PWM down counter
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
void PWMA_IRQHandler(void) {		// PWM interrupt subroutine 
	if (PWMA->PIIR.PWMIF3) {
		CaptureCounter++;						// Delay (PWM_CNR+1) usec
		if (CaptureCounter == 0) {	// Overflow
			}
		PWMA->PIIR.PWMIF0	=	1;			// write 1 to clear this bit to zero
		}
	if (PWMA->CCR2.CAPIF3) {
		if (PWMA->CCR2.CFLRI3) {		// Calculate High Level width
			CaptureValue[0] = CaptureCounter*(PWM_CNR+1)+(PWM_CNR-PWMA->CFLR3);//usec
			CaptureCounter = 0;				// reset
			PWMA->CCR2.CFLRI3 = 0;// write 0 to clear this bit to zero if BCn bit is 0
		}
		if (PWMA->CCR2.CRLRI3) {		//Calculate Low Level width
			CaptureValue[1] = CaptureCounter*(PWM_CNR+1)+(PWM_CNR-PWMA->CRLR3);//usec
			CaptureCounter = 0;				// reset
			PWMA->CCR2.CRLRI3 = 0;// write 0 to clear this bit to zero if BCn bit is 0	
			}
		PWMA->CCR2.CAPIF3 = 1;	// write 1 to clear this bit to zero
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
	//sprintf(lcd1_buffer+7, " %d s.", Timer3Counter);
	//print_lcd(1, lcd1_buffer);
 	
	/* Display capture values */
	if (CaptureValue[0] >= 1000000) {
		sprintf(lcd2_buffer+5, "%dsec", CaptureValue[0]/1000000);
		} else if (CaptureValue[0] >= 1000) {
		sprintf(lcd2_buffer+5, "%dmsec", CaptureValue[0]/1000);
		} else
		sprintf(lcd2_buffer+5, "%dusec", CaptureValue[0]);
	print_lcd(2, lcd2_buffer);

	if (CaptureValue[1] >= 1000000) {
		sprintf(lcd3_buffer+5, "%dsec", CaptureValue[1]/1000000);
		} else if (CaptureValue[1] >= 1000) {
		sprintf(lcd3_buffer+5, "%dmsec", CaptureValue[1]/1000);
		} else
		sprintf(lcd3_buffer+5, "%dusec", CaptureValue[1]);
	print_lcd(3, lcd3_buffer);
	
	TIMER3->TISR.TIF = 1;    		// Write 1 to clear the interrupt flag 
	}
void Init_LED() {
	// initialize GPIO pins
	DrvGPIO_Open(E_GPC, 12, E_IO_OUTPUT); // GPC12 pin set to output mode
	DrvGPIO_Open(E_GPC, 13, E_IO_OUTPUT); // GPC13 pin set to output mode
	DrvGPIO_Open(E_GPC, 14, E_IO_OUTPUT); // GPC14 pin set to output mode
	DrvGPIO_Open(E_GPC, 15, E_IO_OUTPUT); // GPC15 pin set to output mode
	// set GPIO pins to output Low
	DrvGPIO_SetBit(E_GPC, 12); // GPC12 pin output Hi to turn off LED
	DrvGPIO_SetBit(E_GPC, 13); // GPC13 pin output Hi to turn off LED
	DrvGPIO_SetBit(E_GPC, 14); // GPC14 pin output Hi to turn off LED
	DrvGPIO_SetBit(E_GPC, 15); // GPC15 pin output Hi to turn off LED
	}
	void EINT1Callback(void) {
	DrvGPIO_DisableEINT1();
		clr_all_pannal();
		//GPC_12 = 1;
		//GPC_13 = 1;
		//GPC_14 = 1;	
		//GPC_15 = 1;
	if(toggle == 1){
				GPC_12 = 0;
		}
		if(toggle == 2){
				GPC_13 = 0;
		}
		if(toggle == 3){
				GPC_14 = 0;
		}
		if(toggle == 4){
				GPC_15 = 0;
		}
		if(toggle == 5){
				GPC_15 = 1;
		}
		if(toggle == 6){
				GPC_14 = 1;
		}
		if(toggle == 7){
			GPC_13 = 1;	
		}			
		if(toggle == 8){
				GPC_12 = 1;
			toggle = 0;
				}
			toggle=toggle+1;			
	
	DrvSYS_Delay(300000);
	DrvGPIO_EnableEINT1(E_IO_BOTH_EDGE, E_MODE_EDGE, EINT1Callback);
	}
//----------------------------------------------------------------------MAIN
int32_t main() {
	int8_t number;
	uint8_t LCDcolumn = 1;
	uint8_t dataout[1] = "1";

	STR_UART_T sParam;
	STR_UART_T sParam2;

	UNLOCKREG();
	DrvSYS_Open(48000000);
	LOCKREG();
	
	Initial_pannel();  //call initial pannel function
	clr_all_pannal();
	//print_lcd(0,"Smpl_UART0    ");
	
	InitPWM1();
	InitPWM2();
	InitCapture3();
	Init_LED();
	
	InitTIMER3();
   	
	/* Set UART Pin */
	DrvGPIO_InitFunction(E_FUNC_UART0);		
	DrvGPIO_InitFunction(E_FUNC_UART2);
	
	/* Configure general GPIO interrupt */
	DrvGPIO_Open(E_GPB, 15, E_IO_INPUT);

	/* Configure external interrupt */
	DrvGPIO_EnableEINT1(E_IO_BOTH_EDGE, E_MODE_EDGE, EINT1Callback);

	/* UART Setting */
	sParam.u32BaudRate = 9600;
	sParam.u8cDataBits = DRVUART_DATABITS_8;
	sParam.u8cStopBits = DRVUART_STOPBITS_1;
	sParam.u8cParity = DRVUART_PARITY_NONE;
	sParam.u8cRxTriggerLevel = DRVUART_FIFO_1BYTES;
	/* UART Setting */	
	sParam2.u32BaudRate = 9600;
	sParam2.u8cDataBits = DRVUART_DATABITS_8;
	sParam2.u8cStopBits = DRVUART_STOPBITS_1;
	sParam2.u8cParity = DRVUART_PARITY_NONE;
	sParam2.u8cRxTriggerLevel = DRVUART_FIFO_1BYTES;

	/* Set UART Configuration */
 	if (DrvUART_Open(UART_PORT0,&sParam) != E_SUCCESS);  

	//DrvUART_EnableInt(UART_PORT0, DRVUART_RDAINT, UART0_INT_HANDLE);  
	
	/* Set UART Configuration */
 	if (DrvUART_Open(UART_PORT2,&sParam2) != E_SUCCESS);  

	DrvUART_EnableInt(UART_PORT2, DRVUART_RDAINT, UART2_INT_HANDLE); 
	DrvGPIO_ClrBit(E_GPC, 13); 		// Function -> output Low to turn on LED
	DrvGPIO_ClrBit(E_GPC, 14); 		// Function -> output Low to turn on LED
	while (1) {
		number = Scankey();
		
		if (number == 1) {
			dataout[0] = 0x31;
			DrvUART_Write(UART_PORT0, dataout,1);

			Show_Word(0,LCDcolumn,'1');
			Show_Word(0,LCDcolumn+1,' ');
			LCDcolumn++;
				if (LCDcolumn > 14) LCDcolumn = 1;			
			DrvSYS_Delay(DELAY300ms);
			}
		if (number == 2) {
			dataout[0] = 0x32;
			DrvUART_Write(UART_PORT0, dataout,1);

			Show_Word(0,LCDcolumn,'2');
			Show_Word(0,LCDcolumn+1,' ');
			LCDcolumn++;
				if (LCDcolumn > 14) LCDcolumn = 1;			
			DrvSYS_Delay(DELAY300ms);			
			}			
		if (number == 3) {
			dataout[0] = 0x33;
			DrvUART_Write(UART_PORT0, dataout,1);

			Show_Word(0,LCDcolumn,'3');
			Show_Word(0,LCDcolumn+1,' ');
			LCDcolumn++;
				if (LCDcolumn > 14) LCDcolumn = 1;			
			DrvSYS_Delay(DELAY300ms);	
			}
		if (number == 4) {
			dataout[0] = 0x34;
			DrvUART_Write(UART_PORT0, dataout,1);

			Show_Word(0,LCDcolumn,'4');
			Show_Word(0,LCDcolumn+1,' ');
			LCDcolumn++;
				if (LCDcolumn > 14) LCDcolumn = 1;			
			DrvSYS_Delay(DELAY300ms);	
			}
		if (number == 5) {
			dataout[0] = 0x35;
			DrvUART_Write(UART_PORT0, dataout,1);

			Show_Word(0,LCDcolumn,'5');
			Show_Word(0,LCDcolumn+1,' ');
			LCDcolumn++;
				if (LCDcolumn > 14) LCDcolumn = 1;			
			DrvSYS_Delay(DELAY300ms);
			}	
			
			
	//DrvUART_Close(UART_PORT0);
		}
	}