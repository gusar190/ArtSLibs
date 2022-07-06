#include "AuxFunctions.hpp"
#include <cmath>

uint8_t intToStr( int32_t numberIn, char *stringOut ) {

    uint8_t length = getCountOfDigits( numberIn );
    uint8_t counter = 0;
    if( numberIn < 0 ) {
        //length++;
        *stringOut = '-';
        numberIn *= -1;
        length++;
        counter = 1;
    }
    stringOut += length;
    while( counter < length )
    {
        *--stringOut = ( numberIn % 10 ) + '0';
        numberIn /= 10;
        counter++;
    }
    return length;
}

uint8_t doubleToStr( double numberIn, char *stringOut, uint8_t precision = 0 ) {

    uint8_t length = getCountOfDigits( (uint32_t)numberIn );
    uint8_t counter = 0;
    if( numberIn < 0 ) {
        *stringOut = '-';
        numberIn *= -1;
        length++;
        counter++;
    }
    if( precision != 0 ) {
        numberIn *= pow( (double)10, precision );
        length += ( precision + 1 );
        stringOut += length;
        while( precision != 0 ) {
            *--stringOut = ( (uint32_t)numberIn % 10 ) + '0';
            numberIn /= 10;
            counter++;
            precision--;
        }
        
        *--stringOut = '.';
        counter++;
    } else {
        stringOut += length;
    }
    while( counter < length )
    {
        *--stringOut = ( (uint32_t)numberIn % 10 ) + '0';
        numberIn /= 10;
        counter++;
    }
    return length;
}

inline uint8_t getCountOfDigits( int32_t number ) {
    uint8_t length = 0;
    do {
        number /= 10;
        length++;
    }
    while( number );
    return length;
}