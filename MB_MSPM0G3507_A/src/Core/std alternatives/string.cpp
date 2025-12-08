/*
 * string.cpp
 *
 *  Created on: Nov 8, 2025
 *      Author: turtl
 */

#include "string.hpp"

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
