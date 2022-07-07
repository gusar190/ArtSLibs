#ifndef WS0010_HPP
#define WS0010_HPP

//#define NO_BUSY_FLAG_CHECK
//typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

#define WS0010_BUS_WIDTH_4_BIT 4
#define WS0010_BUS_WIDTH_8_BIT 8
#define WS0010_CHARACTER_BUFFER_SIZE 255
//Inititalization method parameters
#define WS0010_DCDC_ON 1
#define WS0010_DCDC_OFF 0
#define WS0010_ON 1
#define WS0010_OFF 0
#define WS0010_SHIFT_ENABLE 1
#define WS0010_SHIFT_DISABLE 0
#define WS0010_CURSOR_ENABLE 1
#define WS0010_CURSOR_DISABLE 0
#define WS0010_CURSOR_BLINKING_ENABLE 1
#define WS0010_CURSOR_BLINKING_DISABLE 0
#define WS0010_CURSOR_DISPLAY_INCREMENT 1
#define WS0010_CURSOR_DISPLAY_DECREMENT 0
#define WS0010_CHARACTER_MODE 0
#define WS0010_GRAPHIC_MODE 1
#define WS0010_CHARACTER_FONT_5X8 0
#define WS0010_CHARACTER_FONT_5X10 1
#define WS0010_FONT_TABLE_EN_JAP 0
#define WS0010_FONT_TABLE_EURO 1
#define WS0010_FONT_TABLE_EN_RU 2
#define WS0010_ONE_LINE 0
#define WS0010_TWO_LINES 1
#define WS0010_DATA_ALIGN_LEFT 0
#define WS0010_DATA_ALIGN_RIGHT 1
#define WS0010_MIN_POS 0
#define WS0010_MAX_POS 128

#include <string.h>
#include "SystemTimer.h"

#if defined USE_HAL_DRIVER
//#if defined MCU=STM32F4xx
	#include "stm32f4xx.h"
	#include "stm32f4xx_hal_rcc.h"
	#include "stm32f4xx_hal_gpio.h"
//endif
#endif
/*
#include "stm32f0xx.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_gpio.h"
*/
#include "DisplayBase.hpp"
#include "AuxFunctions.hpp"
//delete this comment:)
class WS0010 : public DisplayBase {
    private:
    /*Private Vars*/
	SystemTimer timer;
    GPIO_InitTypeDef WS0010_GPIO_InitStructure;
    GPIO_TypeDef *GPIO_EN, *GPIO_RS, *GPIO_RW, *GPIO_DB7, *GPIO_DB6, 
                 *GPIO_DB5, *GPIO_DB4, *GPIO_DB3, *GPIO_DB2, *GPIO_DB1, *GPIO_DB0;
    uint32_t GPIO_Pin_EN, GPIO_Pin_RS, GPIO_Pin_RW, GPIO_Pin_DB7, GPIO_Pin_DB6,
             GPIO_Pin_DB5, GPIO_Pin_DB4, GPIO_Pin_DB3, GPIO_Pin_DB2, 
             GPIO_Pin_DB1, GPIO_Pin_DB0;
    bool GPIOIsInput_FLAG, showTime_FLAG;
    uint8_t busWidth;
    uint8_t buffer[WS0010_CHARACTER_BUFFER_SIZE];
    uint8_t bufferPtr;
    uint8_t bitID, bitS, bitD, bitC, bitB, bitGC, bitPWR;
    uint8_t bitDL, bitN, bitF, bitFT1, bitFT0;
    
    /*Redefined SPL functions*/
    void GPIO_WriteBit( GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t BitVal );
    
    /*Low-level GPIO methods*/
    void enableGPIORCC( GPIO_TypeDef* GPIOx) ;
    void setGPIOAsInput( void );
    void setGPIOAsOutput( void );
    void waitForBusyFlag( void );
    void writeCMD( uint8_t cmd );
    //void writeData( uint8_t data );
    uint8_t readData( void );
    
    public:
    /*Constructor and Destructor*/
    WS0010();

    void setGPIO( GPIO_TypeDef* GPIO_EN, uint32_t GPIO_Pin_EN,
                  GPIO_TypeDef* GPIO_RS, uint32_t GPIO_Pin_RS,
                  GPIO_TypeDef* GPIO_RW, uint32_t GPIO_Pin_RW,
                  GPIO_TypeDef* GPIO_DB7, uint32_t GPIO_Pin_DB7,
                  GPIO_TypeDef* GPIO_DB6, uint32_t GPIO_Pin_DB6,
                  GPIO_TypeDef* GPIO_DB5, uint32_t GPIO_Pin_DB5,
                  GPIO_TypeDef* GPIO_DB4, uint32_t GPIO_Pin_DB4 );
    
    void setGPIO( GPIO_TypeDef* GPIO_EN, uint32_t GPIO_Pin_EN,
                  GPIO_TypeDef* GPIO_RS, uint32_t GPIO_Pin_RS,
                  GPIO_TypeDef* GPIO_RW, uint32_t GPIO_Pin_RW,
                  GPIO_TypeDef* GPIO_DB7, uint32_t GPIO_Pin_DB7,
                  GPIO_TypeDef* GPIO_DB6, uint32_t GPIO_Pin_DB6,
                  GPIO_TypeDef* GPIO_DB5, uint32_t GPIO_Pin_DB5,
                  GPIO_TypeDef* GPIO_DB4, uint32_t GPIO_Pin_DB4,
                  GPIO_TypeDef* GPIO_DB3, uint32_t GPIO_Pin_DB3,
                  GPIO_TypeDef* GPIO_DB2, uint32_t GPIO_Pin_DB2,
                  GPIO_TypeDef* GPIO_DB1, uint32_t GPIO_Pin_DB1,
                  GPIO_TypeDef* GPIO_DB0, uint32_t GPIO_Pin_DB0 );
    
    /*High-level public methods*/
    void init( void );
    void init( uint8_t displayMode, uint8_t numberOfLines,
               uint8_t characterFont, uint8_t characterFontTable,
               uint8_t setCursorDisplayInc, uint8_t setDisplayShift,
               uint8_t setCursor, uint8_t setCursorBlinking,
               uint8_t setDCDC, uint8_t setDisplay );
    void clear( void );
    void returnHome( void );
    void entryModeSet( uint8_t bitID, uint8_t bitS );
    void displayOnOffControl( uint8_t bitD, uint8_t bitC, uint8_t bitB );
    void cursorDisplayShiftModePwr( uint8_t bitSCGC, uint8_t bitRLPWR, uint8_t bitDB1, uint8_t bitDB0 );
    void functionSet( uint8_t bitDL, uint8_t bitN, uint8_t bitF, uint8_t bitFT1, uint8_t bitFT0 );
    void setCGRAMAddress( uint8_t byteACG );
    void setDDRAMAddress( uint8_t byteADD );
    uint8_t readBusyFlagAndAddress( void );
    
    /*API public methods*/
    void on( void );
    void off( void );
    void onDCDC( void );
    void offDCDC( void );
    void setGraphicMode( void );
    void setCharacterMode( void );
    void enableCursor( void );
    void disableCursor( void );
    void enableCursorBlinking( void );
    void disableCursorBlinking( void );
    void setCursorDecrement( void );
    void setCursorIncrement( void );
    void enableShift( void );
    void disableShift( void );
    void moveCursorLeft( void );
    void moveCursorRight( void );
    void shiftLeft( void );
    void shiftLeft( uint8_t steps );
    void shiftRight( void );
    void shiftRight( uint8_t steps );
    void writeData( uint8_t data );//TEMP, move to private after test!!!
    //uint8_t readData( void );//TEMP, move to private after test!!!
    void update( void );
    void sendString( char * str );
    void sendString( char * str, uint16_t pos, uint16_t num, uint16_t spaceLength );
    void sendString( char * str, uint8_t x, uint8_t y, uint16_t num, uint16_t spaceLength );
    void sendNumber( uint8_t xStart, uint8_t xEnd, uint8_t y, int32_t number, uint8_t align );
    void showTime( void );
    void clearBuffer( void );
};

#endif
