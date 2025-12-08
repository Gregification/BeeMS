/*
 * common.c
 *
 *  Created on: Nov 9, 2025
 *      Author: turtl
 */

#include "common.h"

bool arrCmp(void const * a, void const * b, buffersize_t len){
    if(a == 0 || b == 0)
        return false;

    for(buffersize_t i = 0; i < len; i++){
        if(((uint8_t const *)a)[i] != ((uint8_t const *)b)[i])
            return false;
    }

    return true;
}
