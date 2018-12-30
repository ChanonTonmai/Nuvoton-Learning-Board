#include <stdio.h>	
#include <string.h>																										 
#include "NUC1xx.h"
#include "LCD_Driver.h"
#include "lib_pwm.h" 

#define	PWM_CNR	0xFFFF

uint16_t	CaptureCounter = 0;
uint32_t	CaptureValue[2];

void InitPWM0(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPAMFP.PWM0_AD13 = 1;		// System Manager Control Registers
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM01_EN = 1;	// Enable PWM clock
	SYSCLK->CLKSEL1.PWM01_S = 0;	// Select 12MHz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz 
	PWMA->PPR.CP01 = 11;	// Prescaler 0~255, Setting 0 to stop output clock
	PWMA->CSR.CSR0 = 3;		// clock divider ->	0:/2, 1:/4, 2:/8, 3:/16, 4:/1
	// PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]
	// Ex:= 12M/[(11+1)*(16)*(0xFFFF+1)] = 0.95367 Hz -> T = 1.048576 ~ 262+786

	/* Step 3. Select PWM Operation mode */
	PWMA->PCR.CH0MOD = 1;		// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD form 0 to 1.
	PWMA->CNR0 = 0xFFFF; // Set the period counter of the PWM0; 0xFFFF = 65535
	PWMA->CMR0 = 0x3FFF; // Set the ton period counter of the PWM0; 0x3FFF = 32767 
	// CMR < CNR:	PWM low width = (CNR-CMR) unit [one PWM clock cycle]
	//						PWM high width = (CMR+1) unit 

	PWMA->PCR.CH0INV = 0;	// Inverter -> 0:off, 1:on
	PWMA->PCR.CH0EN = 1;	// PWM function -> 0:Disable, 1:Enable
 	PWMA->POE.PWM0 = 1;		// Output to pin -> 0:Diasble, 1:Enable
}

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
void InitPWM3(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPAMFP.PWM3_I2SMCLK = 1;		// System Manager Control Registers
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM23_EN = 1;	// Enable PWM clock
	SYSCLK->CLKSEL1.PWM23_S = 0;	// Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz 
	PWMA->PPR.CP23 = 1;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMA->CSR.CSR3 = 4;		// clock divider ->	0:/2, 1:/4, 2:/8, 3:/16, 4:/1
	// PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]
	// Ex:= 12M/[(1+1)*(1)*(8191)] = 732.5 Hz -> T = 1365 micros. ~ 682+682
	
	/* Step 3. Select PWM Operation mode */
	PWMA->PCR.CH3MOD = 1;	// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD form 0 to 1.
	PWMA->CNR3 = 0x1FFF;	// 0x1FFF = 8191
	PWMA->CMR3 = 0x0FFF;

	PWMA->PCR.CH3INV = 0;	//Inverter->0:off, 1:on
	PWMA->PCR.CH3EN = 1;	//PWM function->0:Disable, 1:Enable
 	PWMA->POE.PWM3 = 1;		//Output to pin->0:Diasble, 1:Enable
}	

void InitPWM4(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPBMFP.TM3_PWM4 = 1;		// System Manager Control Registers
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM45_EN = 1;	// Enable PWM clock
	SYSCLK->CLKSEL2.PWM45_S = 0;	// Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz 
	PWMB->PPR.CP01 = 1;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMB->CSR.CSR0 = 4;		// clock divider ->	0:/2, 1:/4, 2:/8, 3:/16, 4:/1
	// PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]
	// Ex:= 12M/[(1+1)*(1)*(8191)] = 732.5 Hz -> T = 1365 micros. ~ 682+682
	
	/* Step 3. Select PWM Operation mode */
	PWMB->PCR.CH0MOD = 1;	// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD form 0 to 1.
	PWMB->CNR0 = 0x1FFF;	// 0x1FFF = 8191
	PWMB->CMR0 = 0x0FFF;

	PWMB->PCR.CH0INV = 0;	//Inverter->0:off, 1:on
	PWMB->PCR.CH0EN = 1;	//PWM function->0:Disable, 1:Enable
 	PWMB->POE.PWM0 = 1;		//Output to pin->0:Diasble, 1:Enable
}
	
void InitPWM5(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPEMFP.PWM5 = 1;		// System Manager Control Registers
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM45_EN = 1;	// Enable PWM clock
	SYSCLK->CLKSEL2.PWM45_S = 0;	// Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz 
	PWMB->PPR.CP01 = 1;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMB->CSR.CSR1 = 4;		// clock divider ->	0:/2, 1:/4, 2:/8, 3:/16, 4:/1
	// PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]
	// Ex:= 12M/[(1+1)*(1)*(8191)] = 732.5 Hz -> T = 1365 micros. ~ 682+682
	
	/* Step 3. Select PWM Operation mode */
	PWMB->PCR.CH1MOD = 1;	// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD form 0 to 1.
	PWMB->CNR1 = 0x1FFF;	// 0x1FFF = 8191
	PWMB->CMR1 = 0x0FFF;

	PWMB->PCR.CH1INV = 0;	//Inverter->0:off, 1:on
	PWMB->PCR.CH1EN = 1;	//PWM function->0:Disable, 1:Enable
 	PWMB->POE.PWM1 = 1;		//Output to pin->0:Diasble, 1:Enable
}

void InitPWM6(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPEMFP.PWM6 = 1;		// System Manager Control Registers
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM67_EN = 1;	// Enable PWM clock
	SYSCLK->CLKSEL2.PWM67_S = 0;	// Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz 
	PWMB->PPR.CP23 = 1;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMB->CSR.CSR2 = 4;		// clock divider ->	0:/2, 1:/4, 2:/8, 3:/16, 4:/1
	// PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]
	// Ex:= 12M/[(1+1)*(1)*(8191)] = 732.5 Hz -> T = 1365 micros. ~ 682+682
	
	/* Step 3. Select PWM Operation mode */
	PWMB->PCR.CH2MOD = 1;	// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD form 0 to 1.
	PWMB->CNR2 = 0x1FFF;	// 0x1FFF = 8191
	PWMB->CMR2 = 0x0FFF;

	PWMB->PCR.CH2INV = 0;	//Inverter->0:off, 1:on
	PWMB->PCR.CH2EN = 1;	//PWM function->0:Disable, 1:Enable
 	PWMB->POE.PWM2 = 1;		//Output to pin->0:Diasble, 1:Enable
}

void InitPWM7(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPEMFP.PWM7 = 1;		// System Manager Control Registers
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM67_EN = 1;	// Enable PWM clock
	SYSCLK->CLKSEL2.PWM67_S = 0;	// Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz 
	PWMB->PPR.CP23 = 1;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMB->CSR.CSR3 = 4;		// clock divider ->	0:/2, 1:/4, 2:/8, 3:/16, 4:/1
	// PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]
	// Ex:= 12M/[(1+1)*(1)*(8191)] = 732.5 Hz -> T = 1365 micros. ~ 682+682
	
	/* Step 3. Select PWM Operation mode */
	PWMB->PCR.CH3MOD = 1;	// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD form 0 to 1.
	PWMB->CNR3 = 0x1FFF;	// 0x1FFF = 8191
	PWMB->CMR3 = 0x0FFF;

	PWMB->PCR.CH3INV = 0;	//Inverter->0:off, 1:on
	PWMB->PCR.CH3EN = 1;	//PWM function->0:Disable, 1:Enable
 	PWMB->POE.PWM3 = 1;		//Output to pin->0:Diasble, 1:Enable
}

void InitCapture0(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPAMFP.PWM0_AD13 = 1;
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM01_EN = 1; // Enable PWM clock
	SYSCLK->CLKSEL1.PWM01_S = 0; // Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz
	
	PWMA->PPR.CP01 = 11;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMA->CSR.CSR0 = 4;			// clock divider -> 0:/2, 1:/4, 2:/8, 3:/16, 4:/1
									         
	/* Step 3. Select PWM Operation mode */
	PWMA->PCR.CH0MOD = 1;		// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD from 0 to 1.
	PWMA->CNR0 = PWM_CNR;			// Set Reload register
	PWMA->CAPENR = 1;				// Enable Capture function pin
	PWMA->CCR0.CAPCH0EN = 1;// Enable Capture function

	/* Step 4. Set PWM Interrupt */
	PWMA->CCR0.CRL_IE0 = 1;	// Enable Capture rising edge interrupt
	PWMA->CCR0.CFL_IE0 = 1;	// Enable Capture falling edge interrupt
	PWMA->PIER.PWMIE0 = 1;	// Enable PWM interrupt for down-counter equal zero.
	NVIC_EnableIRQ(PWMA_IRQn);  // Enable PWM inturrupt

	/* Step 5. Enable PWM down counter*/
	PWMA->PCR.CH0EN = 1;		// Enable PWM down counter
}
void InitCapture1(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPAMFP.PWM1_AD14 = 1;
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM01_EN = 1; // Enable PWM clock
	SYSCLK->CLKSEL1.PWM01_S = 0; // Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz
	
	PWMA->PPR.CP01 = 11;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMA->CSR.CSR1 = 4;			// clock divider -> 0:/2, 1:/4, 2:/8, 3:/16, 4:/1
									         
	/* Step 3. Select PWM Operation mode */
	PWMA->PCR.CH1MOD = 1;		// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD from 0 to 1.
	PWMA->CNR1 = PWM_CNR;			// Set Reload register
	PWMA->CAPENR = 1;				// Enable Capture function pin
	PWMA->CCR0.CAPCH1EN = 1;// Enable Capture function

	/* Step 4. Set PWM Interrupt */
	PWMA->CCR0.CRL_IE1 = 1;	// Enable Capture rising edge interrupt
	PWMA->CCR0.CFL_IE1 = 1;	// Enable Capture falling edge interrupt
	PWMA->PIER.PWMIE1 = 1;	// Enable PWM interrupt for down-counter equal zero.
	NVIC_EnableIRQ(PWMA_IRQn);  // Enable PWM inturrupt

	/* Step 5. Enable PWM down counter */
	PWMA->PCR.CH1EN = 1;		// Enable PWM down counter
}

void InitCapture2(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPAMFP.PWM2_AD15 = 1;
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM23_EN = 1; // Enable PWM clock
	SYSCLK->CLKSEL1.PWM23_S = 0; // Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz
	
	PWMA->PPR.CP23 = 11;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMA->CSR.CSR2 = 4;			// clock divider -> 0:/2, 1:/4, 2:/8, 3:/16, 4:/1
									         
	/* Step 3. Select PWM Operation mode */
	PWMA->PCR.CH2MOD = 1;		// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD from 0 to 1.
	PWMA->CNR2 = PWM_CNR;			// Set Reload register
	PWMA->CAPENR = 4;				// Enable Capture function pin
	PWMA->CCR2.CAPCH2EN = 1;// Enable Capture function

	/* Step 4. Set PWM Interrupt */
	PWMA->CCR2.CRL_IE2 = 1;	// Enable Capture rising edge interrupt
	PWMA->CCR2.CFL_IE2 = 1;	// Enable Capture falling edge interrupt
	PWMA->PIER.PWMIE2 = 1;	// Enable PWM interrupt for down-counter equal zero.
	NVIC_EnableIRQ(PWMA_IRQn);  // Enable PWM inturrupt

	/* Step 5. Enable PWM down counter*/
	PWMA->PCR.CH2EN = 1;		// Enable PWM down counter
}
void InitCapture3(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPAMFP.PWM3_I2SMCLK = 1;
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM23_EN = 1; // Enable PWM clock
	SYSCLK->CLKSEL1.PWM23_S = 0; // Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz
	
	PWMA->PPR.CP01 = 11;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMA->CSR.CSR0 = 4;			// clock divider -> 0:/2, 1:/4, 2:/8, 3:/16, 4:/1
									         
	/* Step 3. Select PWM Operation mode */
	PWMA->PCR.CH3MOD = 1;		// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD from 0 to 1.
	PWMA->CNR3 = PWM_CNR;			// Set Reload register
	PWMA->CAPENR = 1;				// Enable Capture function pin
	PWMA->CCR2.CAPCH3EN = 1;// Enable Capture function

	/* Step 4. Set PWM Interrupt */
	PWMA->CCR2.CRL_IE3 = 1;	// Enable Capture rising edge interrupt
	PWMA->CCR2.CFL_IE3 = 1;	// Enable Capture falling edge interrupt
	PWMA->PIER.PWMIE3 = 1;	// Enable PWM interrupt for down-counter equal zero.
	NVIC_EnableIRQ(PWMA_IRQn);  // Enable PWM inturrupt

	/* Step 5. Enable PWM down counter*/
	PWMA->PCR.CH3EN = 1;		// Enable PWM down counter
}



void InitCapture4(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPBMFP.TM3_PWM4 = 1;
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM45_EN = 1; // Enable PWM clock
	SYSCLK->CLKSEL2.PWM45_S = 0; // Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz
	
	PWMB->PPR.CP01 = 11;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMB->CSR.CSR0 = 4;			// clock divider -> 0:/2, 1:/4, 2:/8, 3:/16, 4:/1
									         
	/* Step 3. Select PWM Operation mode */
	PWMB->PCR.CH0MOD = 1;		// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD from 0 to 1.
	PWMB->CNR0 = PWM_CNR;			// Set Reload register
	PWMB->CAPENR = 1;				// Enable Capture function pin
	PWMB->CCR0.CAPCH0EN = 1;// Enable Capture function

	/* Step 4. Set PWM Interrupt */
	PWMB->CCR0.CRL_IE0 = 1;	// Enable Capture rising edge interrupt
	PWMB->CCR0.CFL_IE0 = 1;	// Enable Capture falling edge interrupt
	PWMB->PIER.PWMIE0 = 1;	// Enable PWM interrupt for down-counter equal zero.
	NVIC_EnableIRQ(PWMB_IRQn);  // Enable PWM inturrupt

	/* Step 5. Enable PWM down counter*/
	PWMB->PCR.CH0EN = 1;		// Enable PWM down counter
}
void InitCapture5(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPEMFP.PWM5 = 1;
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM45_EN = 1; // Enable PWM clock
	SYSCLK->CLKSEL2.PWM45_S = 0; // Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz
	
	PWMB->PPR.CP01 = 11;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMB->CSR.CSR1 = 4;			// clock divider -> 0:/2, 1:/4, 2:/8, 3:/16, 4:/1
									         
	/* Step 3. Select PWM Operation mode */
	PWMB->PCR.CH1MOD = 1;		// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD from 0 to 1.
	PWMB->CNR1 = PWM_CNR;			// Set Reload register
	PWMB->CAPENR = 1;				// Enable Capture function pin
	PWMB->CCR0.CAPCH1EN = 1;// Enable Capture function

	/* Step 4. Set PWM Interrupt */
	PWMB->CCR0.CRL_IE1 = 1;	// Enable Capture rising edge interrupt
	PWMB->CCR0.CFL_IE1 = 1;	// Enable Capture falling edge interrupt
	PWMB->PIER.PWMIE1 = 1;	// Enable PWM interrupt for down-counter equal zero.
	NVIC_EnableIRQ(PWMB_IRQn);  // Enable PWM inturrupt

	/* Step 5. Enable PWM down counter */
	PWMB->PCR.CH1EN = 1;		// Enable PWM down counter
}

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
void InitCapture7(void) {
 	/* Step 1. GPIO initial */ 
	SYS->GPEMFP.PWM7 = 1;
	
	/* Step 2. Enable and Select PWM clock source*/		
	SYSCLK->APBCLK.PWM67_EN = 1; // Enable PWM clock
	SYSCLK->CLKSEL2.PWM67_S = 0; // Select 12Mhz for PWM clock source
	// 0:12MHz, 1:32.768 kHz, 2:HCLK, 3:22.1184 MHz
	
	PWMB->PPR.CP23 = 11;		// Prescaler 0~255, Setting 0 to stop output clock
	PWMB->CSR.CSR3 = 4;			// clock divider -> 0:/2, 1:/4, 2:/8, 3:/16, 4:/1
									         
	/* Step 3. Select PWM Operation mode */
	PWMB->PCR.CH3MOD = 1;		// 0:One-shot mode, 1:Auto-load mode
	//CNR and CMR will be auto-cleared after setting CH0MOD from 0 to 1.
	PWMB->CNR3 = PWM_CNR;			// Set Reload register
	PWMB->CAPENR = 1;				// Enable Capture function pin
	PWMB->CCR2.CAPCH3EN = 1;// Enable Capture function

	/* Step 4. Set PWM Interrupt */
	PWMB->CCR2.CRL_IE3 = 1;	// Enable Capture rising edge interrupt
	PWMB->CCR2.CFL_IE3 = 1;	// Enable Capture falling edge interrupt
	PWMB->PIER.PWMIE3 = 1;	// Enable PWM interrupt for down-counter equal zero.
	NVIC_EnableIRQ(PWMB_IRQn);  // Enable PWM inturrupt

	/* Step 5. Enable PWM down counter*/
	PWMB->PCR.CH3EN = 1;		// Enable PWM down counter
}




//------------------------------------------------------------------PWMA_IRQ & PWMB_IRQ
void PWMA_IRQHandler(void) {		// PWM interrupt subroutine 
	if (PWMA->PIIR.PWMIF0) {
		CaptureCounter++;						// Delay (PWM_CNR+1) usec
		if (CaptureCounter == 0) {	// Overflow
			}
		PWMA->PIIR.PWMIF0	=	1;			// write 1 to clear this bit to zero
		}
	if (PWMA->CCR0.CAPIF0) {
		if (PWMA->CCR0.CFLRI0) {		// Calculate High Level width
			CaptureValue[0] = CaptureCounter*(PWM_CNR+1)+(PWM_CNR-PWMA->CFLR0);//usec
			CaptureCounter = 0;				// reset
			PWMA->CCR0.CFLRI0 = 0;// write 0 to clear this bit to zero if BCn bit is 0
		}
		if (PWMA->CCR0.CRLRI0) {		//Calculate Low Level width
			CaptureValue[1] = CaptureCounter*(PWM_CNR+1)+(PWM_CNR-PWMA->CRLR0);//usec
			CaptureCounter = 0;				// reset
			PWMA->CCR0.CRLRI0 = 0;// write 0 to clear this bit to zero if BCn bit is 0	
			}
		PWMA->CCR0.CAPIF0 = 1;	// write 1 to clear this bit to zero
		}
}

void PWMB_IRQHandler(void) {		// PWM interrupt subroutine 
	if (PWMB->PIIR.PWMIF2) {
		CaptureCounter++;						// Delay (PWM_CNR+1) usec
		if (CaptureCounter == 0) {	// Overflow
			}
		PWMB->PIIR.PWMIF2	=	1;			// write 1 to clear this bit to zero
		}
	if (PWMB->CCR2.CAPIF2) {
		if (PWMB->CCR2.CFLRI2) {		// Calculate High Level width
			CaptureValue[0] = CaptureCounter*(PWM_CNR+1)+(PWM_CNR-PWMB->CFLR2);//usec
			CaptureCounter = 0;				// reset
			PWMB->CCR2.CFLRI2 = 0;// write 0 to clear this bit to zero if BCn bit is 0
		}
		if (PWMB->CCR2.CRLRI2) {		//Calculate Low Level width
			CaptureValue[1] = CaptureCounter*(PWM_CNR+1)+(PWM_CNR-PWMB->CRLR2);//usec
			CaptureCounter = 0;				// reset
			PWMB->CCR2.CRLRI2 = 0;// write 0 to clear this bit to zero if BCn bit is 0	
			}
		PWMB->CCR2.CAPIF2 = 1;	// write 1 to clear this bit to zero
		}
}

