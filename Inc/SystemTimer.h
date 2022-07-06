#ifndef SYSTEM_TIMER_H
#define SYSTEM_TIMER_H

#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif

 	void SysTick_Handler(void);

#ifdef __cplusplus
}
#endif


class SystemTimer{
	public:
	enum TimerState {STARTED, STOPPED, EXPIRED};
	SystemTimer( void );
	SystemTimer( uint16_t );
	static void tickSysTime( void );
	void set( uint16_t );
	TimerState getState( void );
	TimerState start( uint16_t );

	private:
	TimerState state{STOPPED};
	inline static uint64_t sysTime{0};
	uint64_t timer{0};
};

#endif
