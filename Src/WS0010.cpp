#include "WS0010.hpp"


//#define NO_BUSY_FLAG_CHECK

/*
void WS0010::GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t BitVal)
{
  if (BitVal != 0)
  {
    GPIOx->BSRRL = GPIO_Pin;
  }
  else
  {
    GPIOx->BSRRH = GPIO_Pin ;
  }
}

void WS0010::enableGPIORCC( GPIO_TypeDef* GPIOx ) {
    if( GPIOx == GPIOA ) {
        RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA, ENABLE );
    } else 
        if( GPIOx == GPIOB ) {
            RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB, ENABLE );
        } else
            if( GPIOx == GPIOC ) {
                RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
            } else
            if( GPIOx == GPIOD ) {
                RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD, ENABLE );
            } else
                if( GPIOx == GPIOE ) {
                    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOE, ENABLE );
                } else
                    if( GPIOx == GPIOF ) {
                        RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOF, ENABLE );
                    } else
                        if( GPIOx == GPIOG ) {
                            RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOG, ENABLE );
                        } else
                            if( GPIOx == GPIOH ) {
                                RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOH, ENABLE );
                            } else
                                if( GPIOx == GPIOI ) {
                                    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOI, ENABLE );
                                } else
                                    if( GPIOx == GPIOJ ) {
                                        RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOJ, ENABLE );
                                    } else
                                        if( GPIOx == GPIOK ) {
                                            RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOK, ENABLE );
                                        }
}
*/
void WS0010::GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t BitVal)
{
  if (BitVal != 0)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = GPIO_Pin ;
  }
}

void WS0010::enableGPIORCC( GPIO_TypeDef* GPIOx ) {
    if( GPIOx == GPIOA ) {
        RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOA, ENABLE );
    } else 
        if( GPIOx == GPIOB ) {
            RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE );
        } else
            if( GPIOx == GPIOC ) {
                RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOC, ENABLE );
            } else
            if( GPIOx == GPIOD ) {
                RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOD, ENABLE );
            } else
                if( GPIOx == GPIOE ) {
                    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOE, ENABLE );
                } else
                    if( GPIOx == GPIOF ) {
                        RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOF, ENABLE );
                    }
}

void WS0010::setGPIOAsInput( void ) {
    WS0010_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB7;
    GPIO_Init( GPIO_DB7, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB6;
    GPIO_Init( GPIO_DB6, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB5;
    GPIO_Init( GPIO_DB5, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB4;
    GPIO_Init( GPIO_DB4, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB3;
    GPIO_Init( GPIO_DB3, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB2;
    GPIO_Init( GPIO_DB2, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB1;
    GPIO_Init( GPIO_DB1, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB0;
    GPIO_Init( GPIO_DB0, &WS0010_GPIO_InitStructure );
    GPIOIsInput_FLAG = SET;
}

void WS0010::setGPIOAsOutput( void ) {
    WS0010_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB7;
    GPIO_Init( GPIO_DB7, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB6;
    GPIO_Init( GPIO_DB6, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB5;
    GPIO_Init( GPIO_DB5, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB4;
    GPIO_Init( GPIO_DB4, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB3;
    GPIO_Init( GPIO_DB3, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB2;
    GPIO_Init( GPIO_DB2, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB1;
    GPIO_Init( GPIO_DB1, &WS0010_GPIO_InitStructure );
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_DB0;
    GPIO_Init( GPIO_DB0, &WS0010_GPIO_InitStructure );
    GPIOIsInput_FLAG = RESET;
}

void WS0010::waitForBusyFlag( void ) {
#ifndef NO_BUSY_FLAG_CHECK
{   
    uint8_t busyFlag = 0;
    if( GPIOIsInput_FLAG == RESET ) {
        setGPIOAsInput();
    }
    GPIO_ResetBits( GPIO_RS, GPIO_Pin_RS );
    GPIO_SetBits( GPIO_RW, GPIO_Pin_RW );
    while( 1 ) {
        waitUS( 1 );
        GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
        waitUS( 1 );
        busyFlag = GPIO_ReadInputDataBit( GPIO_DB7, GPIO_Pin_DB7 );
        GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
        if( busWidth == WS0010_BUS_WIDTH_4_BIT ) {
            waitUS( 1 );
            GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
            waitUS( 1 );
            GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
        }
        if( busyFlag == RESET )
                break;
    }
}
#else
{
    while( timer.start(1) != SystemTimer:EXPIRED );
}
#endif
}

void WS0010::writeCMD( uint8_t cmd ) {
    waitForBusyFlag();
    if( GPIOIsInput_FLAG == SET ) {
        setGPIOAsOutput();
    }
    GPIO_ResetBits( GPIO_RS, GPIO_Pin_RS );
    GPIO_ResetBits( GPIO_RW, GPIO_Pin_RW );
    GPIO_WriteBit( GPIO_DB7, GPIO_Pin_DB7, cmd & 0x80 );
    GPIO_WriteBit( GPIO_DB6, GPIO_Pin_DB6, cmd & 0x40 );
    GPIO_WriteBit( GPIO_DB5, GPIO_Pin_DB5, cmd & 0x20 );
    GPIO_WriteBit( GPIO_DB4, GPIO_Pin_DB4, cmd & 0x10 );
    if( busWidth == WS0010_BUS_WIDTH_4_BIT ) {
        GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
        GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
    }
    GPIO_WriteBit( GPIO_DB3, GPIO_Pin_DB3, cmd & 0x08 );
    GPIO_WriteBit( GPIO_DB2, GPIO_Pin_DB2, cmd & 0x04 );
    GPIO_WriteBit( GPIO_DB1, GPIO_Pin_DB1, cmd & 0x02 );
    GPIO_WriteBit( GPIO_DB0, GPIO_Pin_DB0, cmd & 0x01 );
    GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
    while( timer.start(1) != SystemTimer:EXPIRED );
    GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
    while( timer.start(1) != SystemTimer:EXPIRED );
}

void WS0010::writeData( uint8_t data ) {
    waitForBusyFlag();
    if( GPIOIsInput_FLAG == SET ) {
        setGPIOAsOutput();
    }
    GPIO_SetBits( GPIO_RS, GPIO_Pin_RS );
    GPIO_ResetBits( GPIO_RW, GPIO_Pin_RW );
    GPIO_WriteBit( GPIO_DB7, GPIO_Pin_DB7, data & 0x80 );
    GPIO_WriteBit( GPIO_DB6, GPIO_Pin_DB6, data & 0x40 );
    GPIO_WriteBit( GPIO_DB5, GPIO_Pin_DB5, data & 0x20 );
    GPIO_WriteBit( GPIO_DB4, GPIO_Pin_DB4, data & 0x10 );
    if( busWidth == WS0010_BUS_WIDTH_4_BIT ) {
        GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
        GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
    }
    GPIO_WriteBit( GPIO_DB3, GPIO_Pin_DB3, data & 0x08 );
    GPIO_WriteBit( GPIO_DB2, GPIO_Pin_DB2, data & 0x04 );
    GPIO_WriteBit( GPIO_DB1, GPIO_Pin_DB1, data & 0x02 );
    GPIO_WriteBit( GPIO_DB0, GPIO_Pin_DB0, data & 0x01 );
    GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
    while( timer.start(1) != SystemTimer:EXPIRED );
    GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
    while( timer.start(1) != SystemTimer:EXPIRED );
}

uint8_t WS0010::readData( void ) {
    uint8_t data = 0;
    waitForBusyFlag();
    if( GPIOIsInput_FLAG != SET ) {
        setGPIOAsInput();
    }
    GPIO_SetBits( GPIO_RS, GPIO_Pin_RS );
    GPIO_SetBits( GPIO_RW, GPIO_Pin_RW );
    GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
    waitUS( 1 );
    data |= ( GPIO_ReadInputDataBit( GPIO_DB7, GPIO_Pin_DB7 ) << 7 );
    data |= ( GPIO_ReadInputDataBit( GPIO_DB6, GPIO_Pin_DB6 ) << 6 );
    data |= ( GPIO_ReadInputDataBit( GPIO_DB5, GPIO_Pin_DB5 ) << 5 );
    data |= ( GPIO_ReadInputDataBit( GPIO_DB4, GPIO_Pin_DB4 ) << 4 );
    if( busWidth == WS0010_BUS_WIDTH_4_BIT ) {
        GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
        GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
    }
    data |= ( GPIO_ReadInputDataBit( GPIO_DB3, GPIO_Pin_DB3 ) << 3 );
    data |= ( GPIO_ReadInputDataBit( GPIO_DB2, GPIO_Pin_DB2 ) << 2 );
    data |= ( GPIO_ReadInputDataBit( GPIO_DB1, GPIO_Pin_DB1 ) << 1 );
    data |= ( GPIO_ReadInputDataBit( GPIO_DB0, GPIO_Pin_DB0 ) << 0 );
    GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
    waitUS( 1 );
    return data;
}

/*The class constructor sets variables to its default state.
The display configuration is: 8Bit interface, one line, cursor isn't shown,
AC incrementing on each writeData command, display isn't shifting.
Buffer filled with blank symbols.*/
WS0010::WS0010() {
    bitID = SET; bitS = RESET;
    bitD = SET; bitC = RESET; bitB = RESET;
    bitGC = RESET; bitPWR = SET; 
    bitDL = SET; bitN = RESET; bitF = RESET; bitFT1 = RESET; bitFT0 = RESET;
    for(uint8_t i = 0 ; i < WS0010_CHARACTER_BUFFER_SIZE ; i++)
        buffer[i] = ' ';
    bufferPtr = 0;
}

void WS0010::setGPIO( GPIO_TypeDef* _GPIO_EN, uint32_t _GPIO_Pin_EN,
                      GPIO_TypeDef* _GPIO_RS, uint32_t _GPIO_Pin_RS,
                      GPIO_TypeDef* _GPIO_RW, uint32_t _GPIO_Pin_RW,
                      GPIO_TypeDef* _GPIO_DB7, uint32_t _GPIO_Pin_DB7,
                      GPIO_TypeDef* _GPIO_DB6, uint32_t _GPIO_Pin_DB6,
                      GPIO_TypeDef* _GPIO_DB5, uint32_t _GPIO_Pin_DB5,
                      GPIO_TypeDef* _GPIO_DB4, uint32_t _GPIO_Pin_DB4 ) {
    busWidth = WS0010_BUS_WIDTH_4_BIT;
    GPIO_EN = _GPIO_EN;
    GPIO_RS = _GPIO_RS;
    GPIO_RW = _GPIO_RW;
    GPIO_DB7 = _GPIO_DB7;
    GPIO_DB6 = _GPIO_DB6;
    GPIO_DB5 = _GPIO_DB5;
    GPIO_DB4 = _GPIO_DB4;
    GPIO_DB3 = GPIO_DB7;
    GPIO_DB2 = GPIO_DB6;
    GPIO_DB1 = GPIO_DB5;
    GPIO_DB0 = GPIO_DB4;
    GPIO_Pin_EN = _GPIO_Pin_EN;
    GPIO_Pin_RS = _GPIO_Pin_RS;
    GPIO_Pin_RW = _GPIO_Pin_RW;
    GPIO_Pin_DB7 = _GPIO_Pin_DB7;
    GPIO_Pin_DB6 = _GPIO_Pin_DB6;
    GPIO_Pin_DB5 = _GPIO_Pin_DB5;
    GPIO_Pin_DB4 = _GPIO_Pin_DB4;
    GPIO_Pin_DB3 = GPIO_Pin_DB7;
    GPIO_Pin_DB2 = GPIO_Pin_DB6;
    GPIO_Pin_DB1 = GPIO_Pin_DB5;
    GPIO_Pin_DB0 = GPIO_Pin_DB4;
    enableGPIORCC( GPIO_EN );
    enableGPIORCC( GPIO_RS );
    enableGPIORCC( GPIO_RW );
    enableGPIORCC( GPIO_DB7 );
    enableGPIORCC( GPIO_DB6 );
    enableGPIORCC( GPIO_DB5 );
    enableGPIORCC( GPIO_DB4 );
    WS0010_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    WS0010_GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    WS0010_GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//GPIO_PuPd_DOWN;
    WS0010_GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_EN;
    GPIO_Init( GPIO_EN, &WS0010_GPIO_InitStructure);
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_RS;
    GPIO_Init( GPIO_RS, &WS0010_GPIO_InitStructure);
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_RW;
    GPIO_Init( GPIO_RW, &WS0010_GPIO_InitStructure);
    setGPIOAsOutput();
}

void WS0010::setGPIO( GPIO_TypeDef* _GPIO_EN, uint32_t _GPIO_Pin_EN,
                      GPIO_TypeDef* _GPIO_RS, uint32_t _GPIO_Pin_RS,
                      GPIO_TypeDef* _GPIO_RW, uint32_t _GPIO_Pin_RW,
                      GPIO_TypeDef* _GPIO_DB7, uint32_t _GPIO_Pin_DB7,
                      GPIO_TypeDef* _GPIO_DB6, uint32_t _GPIO_Pin_DB6,
                      GPIO_TypeDef* _GPIO_DB5, uint32_t _GPIO_Pin_DB5,
                      GPIO_TypeDef* _GPIO_DB4, uint32_t _GPIO_Pin_DB4,
                      GPIO_TypeDef* _GPIO_DB3, uint32_t _GPIO_Pin_DB3,
                      GPIO_TypeDef* _GPIO_DB2, uint32_t _GPIO_Pin_DB2,
                      GPIO_TypeDef* _GPIO_DB1, uint32_t _GPIO_Pin_DB1,
                      GPIO_TypeDef* _GPIO_DB0, uint32_t _GPIO_Pin_DB0 ) {
    busWidth = WS0010_BUS_WIDTH_8_BIT;
    GPIO_EN = _GPIO_EN;
    GPIO_RS = _GPIO_RS;
    GPIO_RW = _GPIO_RW;
    GPIO_DB7 = _GPIO_DB7;
    GPIO_DB6 = _GPIO_DB6;
    GPIO_DB5 = _GPIO_DB5;
    GPIO_DB4 = _GPIO_DB4;
    GPIO_DB3 = _GPIO_DB3;
    GPIO_DB2 = _GPIO_DB2;
    GPIO_DB1 = _GPIO_DB1;
    GPIO_DB0 = _GPIO_DB0;
    GPIO_Pin_EN = _GPIO_Pin_EN;
    GPIO_Pin_RS = _GPIO_Pin_RS;
    GPIO_Pin_RW = _GPIO_Pin_RW;
    GPIO_Pin_DB7 = _GPIO_Pin_DB7;
    GPIO_Pin_DB6 = _GPIO_Pin_DB6;
    GPIO_Pin_DB5 = _GPIO_Pin_DB5;
    GPIO_Pin_DB4 = _GPIO_Pin_DB4;
    GPIO_Pin_DB3 = _GPIO_Pin_DB3;
    GPIO_Pin_DB2 = _GPIO_Pin_DB2;
    GPIO_Pin_DB1 = _GPIO_Pin_DB1;
    GPIO_Pin_DB0 = _GPIO_Pin_DB0;
    enableGPIORCC( GPIO_EN );
    enableGPIORCC( GPIO_RS );
    enableGPIORCC( GPIO_RW );
    enableGPIORCC( GPIO_DB7 );
    enableGPIORCC( GPIO_DB6 );
    enableGPIORCC( GPIO_DB5 );
    enableGPIORCC( GPIO_DB4 );
    enableGPIORCC( GPIO_DB3 );
    enableGPIORCC( GPIO_DB2 );
    enableGPIORCC( GPIO_DB1 );
    enableGPIORCC( GPIO_DB0 );
    WS0010_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    WS0010_GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    WS0010_GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    WS0010_GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_EN;
    GPIO_Init( GPIO_EN, &WS0010_GPIO_InitStructure);
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_RS;
    GPIO_Init( GPIO_RS, &WS0010_GPIO_InitStructure);
    WS0010_GPIO_InitStructure.GPIO_Pin = GPIO_Pin_RW;
    GPIO_Init( GPIO_RW, &WS0010_GPIO_InitStructure);
    setGPIOAsOutput();
}

void WS0010::init( void ) {
    functionSet( bitDL, bitN, bitF, bitFT1, bitFT0 );
    displayOnOffControl( bitD, bitC, bitB );
    clear();
    entryModeSet( bitID, bitS );
}

void WS0010::init( uint8_t displayMode, uint8_t numberOfLines,
               uint8_t characterFont, uint8_t characterFontTable,
               uint8_t setCursorDisplayInc, uint8_t setDisplayShift,
               uint8_t setCursor, uint8_t setCursorBlinking,
               uint8_t setDCDC, uint8_t setDisplay ) {
    if( busWidth == WS0010_BUS_WIDTH_8_BIT ) {
        bitDL = 1;
    } else {
        bitDL = 0;
    }
    bitN = numberOfLines;
    bitF = characterFont;
    bitFT1 = characterFontTable >> 1;
    bitFT0 = characterFontTable & 0x01;
    bitID = setCursorDisplayInc;
    bitS = setDisplayShift;
    bitC = setCursor;
    bitB = setCursorBlinking;
    bitGC = displayMode;
    bitPWR = setDCDC;
    while( timer.start(50) != SystemTimer:EXPIRED );//PowerOn delay
    functionSet( bitDL, bitN, bitF, bitFT1, bitFT0 );
    displayOnOffControl( 0, bitC, bitB );//Switch off display
    cursorDisplayShiftModePwr( bitGC, bitPWR, 1, 1 );//added 12.12.2018
    clear();
    returnHome();
    entryModeSet( bitID, bitS );
    bitD = setDisplay;
    displayOnOffControl( bitD, bitC, bitB );
}

void WS0010::clear( void ) {
    writeCMD( 0x01 );
    while( timer.start(10) != SystemTimer:EXPIRED );
}

void WS0010::returnHome( void ) {
    writeCMD( 0x02 );
}

void WS0010::entryModeSet( uint8_t _bitID, uint8_t _bitS ) {
    bitID = _bitID;
    bitS = _bitS;
    writeCMD( 0x04 | (bitID << 1) | (bitS << 0) );
}

void WS0010::displayOnOffControl( uint8_t _bitD, uint8_t _bitC, uint8_t _bitB ) {
    bitD = _bitD;
    bitC = _bitC;
    bitB = _bitB;
    writeCMD( 0x08 | (bitD << 2) | (bitC << 1) | (bitB << 0) );
}

void WS0010::cursorDisplayShiftModePwr( uint8_t _bitSCGC, uint8_t _bitRLPWR, uint8_t _bitDB1, uint8_t _bitDB0 ) {
    bitGC = _bitSCGC;
    bitPWR = _bitRLPWR;
    //bitDB1 puts on place of bitDB0 to avoid possible programmer error conditions 01 or 10;
    writeCMD( 0x10 | (bitGC << 3) | (bitPWR << 2) | (_bitDB1 << 1) | (_bitDB1 << 0) );
}

void WS0010::functionSet( uint8_t _bitDL, uint8_t _bitN, uint8_t _bitF, uint8_t _bitFT1, uint8_t _bitFT0 ) {
    bitDL = _bitDL;
    bitN = _bitN;
    bitF = _bitF;
    bitFT1 = _bitFT1;
    bitFT0 = _bitFT0;
    switch( busWidth ) {
    case WS0010_BUS_WIDTH_4_BIT: {
        //waitForBusyFlag();
        if( GPIOIsInput_FLAG == SET ) {
            setGPIOAsOutput();
        }
        GPIO_ResetBits( GPIO_RS, GPIO_Pin_RS );
        GPIO_ResetBits( GPIO_RW, GPIO_Pin_RW );
        for( uint8_t i = 0 ; i < 5 ; i++) {
            GPIO_WriteBit( GPIO_DB7, GPIO_Pin_DB7, 0 );
            GPIO_WriteBit( GPIO_DB6, GPIO_Pin_DB6, 0 );
            GPIO_WriteBit( GPIO_DB5, GPIO_Pin_DB5, 0 );
            GPIO_WriteBit( GPIO_DB4, GPIO_Pin_DB4, 0 );
            GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
            while( timer.start(1) != SystemTimer:EXPIRED );
            GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
            while( timer.start(1) != SystemTimer:EXPIRED );
        }
        for( uint8_t i = 0 ; i < 2 ; i++) {
            GPIO_WriteBit( GPIO_DB7, GPIO_Pin_DB7, 0 );
            GPIO_WriteBit( GPIO_DB6, GPIO_Pin_DB6, 0 );
            GPIO_WriteBit( GPIO_DB5, GPIO_Pin_DB5, 1 );
            GPIO_WriteBit( GPIO_DB4, GPIO_Pin_DB4, 0 );
            GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
            while( timer.start(1) != SystemTimer:EXPIRED );
            GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
            while( timer.start(1) != SystemTimer:EXPIRED );
        }
        GPIO_WriteBit( GPIO_DB7, GPIO_Pin_DB7, bitN );
        GPIO_WriteBit( GPIO_DB6, GPIO_Pin_DB6, bitF );
        GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
        GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
        
        for( uint8_t i = 0 ; i < 4 ; i++) {
            GPIO_WriteBit( GPIO_DB7, GPIO_Pin_DB7, 0 );
            GPIO_WriteBit( GPIO_DB6, GPIO_Pin_DB6, 0 );
            GPIO_WriteBit( GPIO_DB5, GPIO_Pin_DB5, 0 );
            GPIO_WriteBit( GPIO_DB4, GPIO_Pin_DB4, 0 );
            GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
            while( timer.start(1) != SystemTimer:EXPIRED );
            GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
            while( timer.start(1) != SystemTimer:EXPIRED );
        }
        for( uint8_t i = 0 ; i < 1 ; i++) {
            GPIO_WriteBit( GPIO_DB7, GPIO_Pin_DB7, 0 );
            GPIO_WriteBit( GPIO_DB6, GPIO_Pin_DB6, 0 );
            GPIO_WriteBit( GPIO_DB5, GPIO_Pin_DB5, 1 );
            GPIO_WriteBit( GPIO_DB4, GPIO_Pin_DB4, 0 );
            GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
            while( timer.start(1) != SystemTimer:EXPIRED );
            GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
            while( timer.start(1) != SystemTimer:EXPIRED );
        }
        GPIO_WriteBit( GPIO_DB7, GPIO_Pin_DB7, bitN );
        GPIO_WriteBit( GPIO_DB6, GPIO_Pin_DB6, bitF );
        GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
        GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
    }break;
    case WS0010_BUS_WIDTH_8_BIT: {
        writeCMD( 0x20 | (bitDL << 4) | (bitN << 3) | (bitF << 2) | (bitFT1 << 1) | (bitFT0 << 0) );
    }break;
    }
}

void WS0010::setCGRAMAddress( uint8_t ACG ) {
    writeCMD( 0x40 | ACG );
}

void WS0010::setDDRAMAddress( uint8_t ADD ) {
    writeCMD( 0x80 | ADD );
}

uint8_t WS0010::readBusyFlagAndAddress( void ) {
    uint8_t address = 0;
    if( GPIOIsInput_FLAG != SET ) {
        setGPIOAsInput();
    }
    GPIO_ResetBits( GPIO_RS, GPIO_Pin_RS );
    GPIO_SetBits( GPIO_RW, GPIO_Pin_RW );
    while( timer.start(1) != SystemTimer:EXPIRED );
    GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
    while( timer.start(1) != SystemTimer:EXPIRED );
    address |= ( GPIO_ReadInputDataBit( GPIO_DB7, GPIO_Pin_DB7 ) << 7 );
    address |= ( GPIO_ReadInputDataBit( GPIO_DB6, GPIO_Pin_DB6 ) << 6 );
    address |= ( GPIO_ReadInputDataBit( GPIO_DB5, GPIO_Pin_DB5 ) << 5 );
    address |= ( GPIO_ReadInputDataBit( GPIO_DB4, GPIO_Pin_DB4 ) << 4 );
    if( busWidth == WS0010_BUS_WIDTH_4_BIT ) {
        GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
        GPIO_SetBits( GPIO_EN, GPIO_Pin_EN );
        while( timer.start(1) != SystemTimer:EXPIRED );
    }
    address |= ( GPIO_ReadInputDataBit( GPIO_DB3, GPIO_Pin_DB3 ) << 3 );
    address |= ( GPIO_ReadInputDataBit( GPIO_DB2, GPIO_Pin_DB2 ) << 2 );
    address |= ( GPIO_ReadInputDataBit( GPIO_DB1, GPIO_Pin_DB1 ) << 1 );
    address |= ( GPIO_ReadInputDataBit( GPIO_DB0, GPIO_Pin_DB0 ) << 0 );
    GPIO_ResetBits( GPIO_EN, GPIO_Pin_EN );
    while( timer.start(1) != SystemTimer:EXPIRED );
    return address;
}

void WS0010::on( void ) {
    bitD = SET;
    displayOnOffControl( bitD, bitC, bitB );
}

void WS0010::off( void ) {
    bitD = RESET;
    displayOnOffControl( bitD, bitC, bitB );
}

void WS0010::onDCDC( void ) {
    bitPWR = SET;
    cursorDisplayShiftModePwr( bitGC, bitPWR, 1, 1 );
}

void WS0010::offDCDC( void ) {
    bitPWR = RESET;
    cursorDisplayShiftModePwr( bitGC, bitPWR, 1, 1 );
}

void WS0010::setGraphicMode( void ) {
    off();
    bitGC = SET;
    cursorDisplayShiftModePwr( bitGC, bitPWR, 1, 1 );
    clear();
    on();
}

void WS0010::setCharacterMode( void ) {
    off();
    bitGC = RESET;
    cursorDisplayShiftModePwr( bitGC, bitPWR, 1, 1 );
    clear();
    on();
}

void WS0010::enableCursor( void ) {
    bitC = SET;
    displayOnOffControl( bitD, bitC, bitB );
}

void WS0010::disableCursor( void ) {
    bitC = RESET;
    displayOnOffControl( bitD, bitC, bitB );
}

void WS0010::enableCursorBlinking( void ) {
    bitB = SET;
    displayOnOffControl( bitD, bitC, bitB );
}

void WS0010::disableCursorBlinking( void ) {
    bitB = RESET;
    displayOnOffControl( bitD, bitC, bitB );
}

void WS0010::setCursorDecrement( void ) {
    bitID = RESET;
    entryModeSet( bitID, bitS );
}

void WS0010::setCursorIncrement( void ) {
    bitID = SET;
    entryModeSet( bitID, bitS );
}

void WS0010::enableShift( void ) {
    bitS = SET;
    entryModeSet( bitID, bitS );
}

void WS0010::disableShift( void ) {
    bitS = RESET;
    entryModeSet( bitID, bitS );
}

void WS0010::moveCursorLeft( void ) {
    cursorDisplayShiftModePwr( 0, 0, 0, 0 );
}

void WS0010::moveCursorRight( void ) {
    cursorDisplayShiftModePwr( 0, 1, 0, 0 );
}

void WS0010::shiftLeft( void ) {
    bitD = RESET;
    displayOnOffControl( bitD, bitC, bitB );
    cursorDisplayShiftModePwr( 1, 0, 0, 0 );
    bitD = SET;
    displayOnOffControl( bitD, bitC, bitB );
}

void WS0010::shiftLeft( uint8_t steps ) {
    bitD = RESET;
    displayOnOffControl( bitD, bitC, bitB );
    while( steps > 0 ) {
        cursorDisplayShiftModePwr( 1, 0, 0, 0 );
        steps--;
    }
    bitD = SET;
    displayOnOffControl( bitD, bitC, bitB );
}

void WS0010::shiftRight( void ) {
    bitD = RESET;
    displayOnOffControl( bitD, bitC, bitB );
    cursorDisplayShiftModePwr( 1, 1, 0, 0 );
    bitD = SET;
    displayOnOffControl( bitD, bitC, bitB );
}

void WS0010::shiftRight( uint8_t steps ) {
    bitD = RESET;
    displayOnOffControl( bitD, bitC, bitB );
    while( steps > 0 ) {
        cursorDisplayShiftModePwr( 1, 1, 0, 0 );
        steps--;
    }
    bitD = SET;
    displayOnOffControl( bitD, bitC, bitB );
}

/*The function updates display contents if some difference between DDRAM
and buffer had been found. It should be placed in main loop*/
void WS0010::update( void ) {
    uint8_t startPos = readBusyFlagAndAddress() & 0x7F;
    int8_t pos = startPos;
    char convertedSymbol;
    /*
    for(uint8_t i = 0 ; i < 128 ; i++) {
        convertedSymbol = buffer[pos];
        switch( buffer[pos] ) {
        case '�': convertedSymbol = 'A'; break;
        case '�': convertedSymbol = 0xA0; break;
        case '�': convertedSymbol = 'B'; break;
        case '�': convertedSymbol = 0xA1; break;
        case '�': convertedSymbol = 0xE0; break;
        case '�': convertedSymbol = 'E'; break;
        case '�': convertedSymbol = 0xA2; break;
        case '�': convertedSymbol = 0xA3; break;
        case '�': convertedSymbol = 0xA4; break;
        case '�': convertedSymbol = 0xA5; break;
        case '�': convertedSymbol = 0xA6; break;
        case '�': convertedSymbol = 'K'; break;
        case '�': convertedSymbol = 0xA7; break;
        case '�': convertedSymbol = 'M'; break;
        case '�': convertedSymbol = 'H'; break;
        case '�': convertedSymbol = 'O'; break;
        case '�': convertedSymbol = 0xA8; break;
        case '�': convertedSymbol = 'P'; break;
        case '�': convertedSymbol = 'C'; break;
        case '�': convertedSymbol = 'T'; break;
        case '�': convertedSymbol = 0xA9; break;
        case '�': convertedSymbol = 0xAA; break;
        case '�': convertedSymbol = 'X'; break;
        case '�': convertedSymbol = 0xE1; break;
        case '�': convertedSymbol = 0xAB; break;
        case '�': convertedSymbol = 0xAC; break;
        case '�': convertedSymbol = 0xE2; break;
        case '�': convertedSymbol = 0xAD; break;
        case '�': convertedSymbol = 0xAE; break;
        case '�': convertedSymbol = 0xAF; break;
        case '�': convertedSymbol = 0xB0; break;
        case '�': convertedSymbol = 0xB1; break;
        case '�': convertedSymbol = 'a'; break;
        case '�': convertedSymbol = 0xB2; break;
        case '�': convertedSymbol = 0xB3; break;
        case '�': convertedSymbol = 0xB4; break;
        case '�': convertedSymbol = 0xE3; break;
        case '�': convertedSymbol = 'e'; break;
        case '�': convertedSymbol = 0xB5; break;
        case '�': convertedSymbol = 0xB6; break;
        case '�': convertedSymbol = 0xB7; break;
        case '�': convertedSymbol = 0xB8; break;
        case '�': convertedSymbol = 0xB9; break;
        case '�': convertedSymbol = 0xBA; break;
        case '�': convertedSymbol = 0xBB; break;
        case '�': convertedSymbol = 0xBC; break;
        case '�': convertedSymbol = 0xBD; break;
        case '�': convertedSymbol = 'o'; break;
        case '�': convertedSymbol = 0xBE; break;
        case '�': convertedSymbol = 'p'; break;
        case '�': convertedSymbol = 'c'; break;
        case '�': convertedSymbol = 0xBF; break;
        case '�': convertedSymbol = 'y'; break;
        case '�': convertedSymbol = 0xE4; break;
        case '�': convertedSymbol = 'x'; break;
        case '�': convertedSymbol = 0xE5; break;
        case '�': convertedSymbol = 0xC0; break;
        case '�': convertedSymbol = 0xC1; break;
        case '�': convertedSymbol = 0xE6; break;
        case '�': convertedSymbol = 0xC2; break;
        case '�': convertedSymbol = 0xC3; break;
        case '�': convertedSymbol = 0xC4; break;
        case '�': convertedSymbol = 0xC5; break;
        case '�': convertedSymbol = 0xC6; break;
        case '�': convertedSymbol = 0xC7; break;
        case '~': convertedSymbol = 0xE9; break;
        case '�': convertedSymbol = 0xCC; break;
        default: {}break;
        }*/
        switch( bitID ) {
        case 1: {
            if( convertedSymbol != readData() ) {
                cursorDisplayShiftModePwr( 0, 0, 0, 0 );//Move cursor one step back
                writeData( convertedSymbol );
            }
            if( ++pos < 0 ) {
                pos = 0;
            }
            }break;
        case 0: {
            if( convertedSymbol != readData() ) {
                cursorDisplayShiftModePwr( 0, 1, 0, 0 );//Move cursor one step forward
                writeData( convertedSymbol );
            }
            if( --pos < 0 ) {
                pos = 127;
            }
            }break;
        };
    }
}

/*The functions puts string into buffer on pos*/

void WS0010::sendString( char * str ) {
    /*!!Works only with null-terminated strings!!*/
    memcpy(&buffer[bufferPtr], str, strlen( str ));
    bufferPtr += strlen( str );
}

void WS0010::sendString( char * str, uint16_t pos, uint16_t num, uint16_t spaceLength ) {
    memset( &buffer[pos], ' ', spaceLength );
    memcpy( &buffer[pos], str, num );
}

void WS0010::sendString( char * str, uint8_t x, uint8_t y, uint16_t num, uint16_t spaceLength ) {
    uint8_t pos = 0;
    switch( y ) {
    case 0: {
        pos = x;
    }break;
    case 1: {
        pos = x + 0x40;
    }break;
    }
    memset( &buffer[pos], ' ', spaceLength );
    memcpy( &buffer[pos], str, num );
}
/**/
void WS0010::sendNumber( uint8_t xStart, uint8_t xEnd, uint8_t y, int32_t number, uint8_t align ) {
    char numberString[11]; //for 32bit signed decimal number
    uint8_t numberLength = intToStr( number, numberString );
    if( y == 1) {
        xStart += 0x40;
        xEnd += 0x40;
    }
    switch( align ) {
    case WS0010_DATA_ALIGN_LEFT: {
        memset( &buffer[xStart], ' ', xEnd - xStart);
        memcpy( &buffer[xStart], numberString, numberLength );
    } break;
    case WS0010_DATA_ALIGN_RIGHT: {
        //memset( &buffer[xStart + numberLength], ' ', xEnd - xStart - numberLength );
        memset( &buffer[xStart], ' ', numberLength );
        memcpy( &buffer[xEnd - numberLength], numberString, numberLength );
    } break;
    default: {} break;
    }
}


void WS0010::showTime( void ) {
    memset( &buffer, ' ', 128 );
    buffer[0] = sysTimeSS/10 + 48;
    buffer[1] = sysTimeSS%10 + 48;
    buffer[2] = ':';
    buffer[3] = sysTimeMM/10 + 48;
    buffer[4] = sysTimeMM%10 + 48;
    buffer[5] = ':';
    buffer[6] = sysTimeHH/10 + 48;
    buffer[7] = sysTimeHH%10 + 48;
    returnHome();
    update();
}

void WS0010::clearBuffer( void ) {
    memset( &buffer, ' ', 128 );
}
