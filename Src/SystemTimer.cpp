#include "SystemTimer.h"

SystemTimer::SystemTimer( void ){

}

SystemTimer::SystemTimer( uint16_t time ){
	set( time );
}

void SystemTimer::tickSysTime( void ){
	sysTime++;
}

inline void SystemTimer::set( uint16_t time ){
	timer = time;
	state = STOPPED;
}

SystemTimer::TimerState SystemTimer::start( uint16_t time = 0 ){
	switch( getState() ){
	case STARTED: return STARTED;
	case STOPPED: {
		if( time == 0 ){
				timer += sysTime;
			} else {
				timer = sysTime+ time;
			}
		state = STARTED;
		return STOPPED;
	}
	case EXPIRED: {
			if( time == 0 ){
					timer += sysTime;
				} else {
					timer = sysTime+ time;
				}
			state = STARTED;
			return EXPIRED;
		}
	}
	return state; //unreachable
}

inline SystemTimer::TimerState SystemTimer::getState( void ){
	if( state == STARTED )
	{
		if( sysTime >= timer ){
			state = EXPIRED;
		}
	}
	return state;
}

/*
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
*/
/*
void SysTick_Handler( void )
{ 
	SoftTimer::tickSysTime();
}
*/
