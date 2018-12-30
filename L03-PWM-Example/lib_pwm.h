#ifndef LIB_PWM_H
#define LIB_PWM_H

void InitPWM0(void);
void InitPWM1(void);
void InitPWM2(void);
void InitPWM3(void);
void InitPWM4(void);
void InitPWM5(void);
void InitPWM6(void);
void InitPWM7(void);

void InitCapture0(void);
void InitCapture1(void);
void InitCapture2(void);
void InitCapture3(void);
void InitCapture4(void);
void InitCapture5(void);
void InitCapture6(void);
void InitCapture7(void);

void PWMA_IRQHandler(void);
void PWMB_IRQHandler(void);

#endif

