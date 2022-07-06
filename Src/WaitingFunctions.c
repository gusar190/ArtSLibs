#include "WaitingFunctions.h"

uint16_t sysTimeUS = 0;                //��������� ����� (������� �����������)
uint16_t sysTimerUS = 0;                //��������� ������ (������� �����������)
uint16_t sysTimeMS = 0;                //��������� ����� (������������)
uint16_t sysTimerMS = 0;                //��������� ������ (������������)
uint16_t sysTimeSS = 0;                //��������� ����� (�������)
uint16_t sysTimerSS = 0;                //��������� ������ (�������)
uint8_t sysTimeMM = 0;                //��������� ����� (������)
uint8_t sysTimeHH = 0;                //��������� ����� (����)
uint8_t sysTimeDD = 0;                //��������� ����� (���)
//uint16_t timerUS = 0;                   //�������������� ������

void waitUS( uint16_t delayTimeUS) {
    uint16_t startTimeUS = sysTimerUS;
    if( startTimeUS + delayTimeUS > 0xFFFF ) {
        startTimeUS = 0xFFFF - startTimeUS;
    }
    while( sysTimerUS < startTimeUS + delayTimeUS ) {
    }
}

void waitMS( uint16_t delayTimeMS) {
    uint16_t endTimeMS;
    if( sysTimerMS + delayTimeMS > 0xFFFF ) {
        endTimeMS = delayTimeMS - (0xFFFF - sysTimerMS);
    } else {
        endTimeMS = sysTimerMS + delayTimeMS;
    }
    while( sysTimerMS != endTimeMS ) {
    }
}

void waitSS( uint16_t delayTimeSS) {
    uint16_t startTimeSS = sysTimerSS;
    if( startTimeSS + delayTimeSS > 0xFFFF ) {
        startTimeSS = 0xFFFF - startTimeSS;
    }
    while( sysTimerSS < startTimeSS + delayTimeSS ) {
    }
}

void waitMM( uint8_t delayTimeMM) {
    uint8_t startTimeMM = sysTimeMM;
    while( sysTimeMM < startTimeMM + delayTimeMM ) {
    }
}

void waitHH( uint8_t delayTimeHH) {
    uint8_t startTimeHH = sysTimeHH;
    while( sysTimeHH < startTimeHH + delayTimeHH ) {
    }
}

void waitDD( uint8_t delayTimeDD) {
    uint8_t startTimeDD = sysTimeDD;
    while( sysTimeDD < startTimeDD + delayTimeDD ) {
    }
}

void SysTick_Handler( void )
{ 
    if( sysTimeUS < 99 ) {
        sysTimeUS++;
        if( sysTimerUS < 0xFFFF ) {
            sysTimerUS++;
        } else {
            sysTimerUS = 0;
        }
    } else {
        sysTimeUS = 0;
        if( sysTimeMS < 999 ) {
            sysTimeMS++;
            if( sysTimerMS < 0xFFFF ) {
                sysTimerMS++;
            } else {
                sysTimerMS = 0;
            }
        } else {                                 
            sysTimeMS = 0;           
            if( sysTimeSS < 59 ) {
                sysTimeSS++;
                if( sysTimerSS < 0xFFFF ) {
                    sysTimerSS++;
                } else {
                    sysTimerSS = 0;
                }
                GPIOF->ODR ^= GPIO_Pin_6;
            } else {                               //���� ������ 60 ������
                sysTimeSS = 0;          //�������� ������� ������
                if( sysTimeMM < 59 ) {      //���� ������ ������ 60 �����
                    sysTimeMM++;          //��������� �����
                } else {                              //���� ������ 60 �����
                    sysTimeMM = 0;        //�������� ������� �����
                    if( sysTimeHH < 23 ) {   //���� ������ ������ 24 �����
                        sysTimeHH++;        //��������� �����
                    } else {                          //���� ������ 24 ����
                        sysTimeHH = 0;      //�������� ������� �����
                    }
                }
            }
        }
    }
}