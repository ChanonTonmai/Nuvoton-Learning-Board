/*--------------------------------------------------------------------------*/
/*                                                                          */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.              */
/* edited: dec 2018: chanon.khong@mail.kmutt.ac.th                          */
/*--------------------------------------------------------------------------*/

#ifndef LIB_TIMER_H
#define LIB_TIMER_H

void InitTIMER0(void);
void TMR0_IRQHandler(void);

void InitTIMER1(void);
void TMR1_IRQHandler(void);	

void InitTIMER2(void);
void TMR2_IRQHandler(void);

void InitTIMER3(void);
void TMR3_IRQHandler(void);	


#endif