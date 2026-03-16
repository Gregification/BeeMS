/*
 * string.cpp
 *
 *  Created on: Nov 8, 2025
 *      Author: turtl
 */

#include "string.hpp"
#include "stdint.h"

buffersize_t ALT::srtCpy(char * to, buffersize_t maxLen, char const * from) {
    buffersize_t i;
    for(i = 0; i < maxLen; i++){
        to[i] = from[i];
        if(from[i] == '\0')
            break;
    }
    to[i] = '\0';
    return i;
}

void ALT::memcpy(void const * from, void * to, buffersize_t len) {
    for(buffersize_t i = 0; i < len; i++)
        ((uint8_t *)to)[i] = ((uint8_t *)from)[i];
}
