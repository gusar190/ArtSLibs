#ifndef WAITING_FUNCTIONS_H
#define WAITING_FUNCTIONS_H

#if defined USE_HAL_DRIVER
	#include "stm32f4xx.h"
	#include "stm32f4xx_it.h"
	#include "stm32f4xx_gpio.h"
*/
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"
#include "stm32f0xx_gpio.h"

extern uint16_t sysTimeUS;                //��������� ����� (������� �����������)
extern uint16_t sysTimerUS;                //��������� ������ (������� �����������)
extern uint16_t sysTimeMS;                //��������� ����� (������������)
extern uint16_t sysTimerMS;                //��������� ������ (������������)
extern uint16_t sysTimeSS;                //��������� ����� (�������)
extern uint16_t sysTimerSS;                //��������� ������ (�������)
extern uint8_t sysTimeMM;                //��������� ����� (������)
extern uint8_t sysTimeHH;                //��������� ����� (����)
extern uint8_t sysTimeDD;                //��������� ����� (���)

void waitUS( uint16_t delayTimeUS);
void waitMS( uint16_t delayTimeMS);
void waitSS( uint16_t delayTimeSS);
void waitMM( uint8_t delayTimeMM);
void waitHH( uint8_t delayTimeHH);
void waitDD( uint8_t delayTimeDD);

#endif
