#ifndef AUX_FUNCTIONS_HPP
#define AUX_FUNCTIONS_HPP

#include <stdlib.h>
#include <stdint.h>

uint8_t intToStr( int32_t numberIn, char *stringOut );
uint8_t doubleToStr( double numberIn, char *stringOut, uint8_t precision );
inline uint8_t getCountOfDigits( int32_t number );

#endif