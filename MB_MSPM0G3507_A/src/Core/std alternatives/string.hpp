/*
 * string.hpp
 *
 *  Created on: Nov 8, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_STD_ALTERNATIVES_STRING_HPP_
#define SRC_CORE_STD_ALTERNATIVES_STRING_HPP_

#include "Core/common.h"


namespace ALT {
    /**
     * returns length of copied content
     */
    buffersize_t srtCpy(char * to, buffersize_t maxLen, char const * from);

    //buffersize_t strLen(char const * str, buffersize_t maxLen);
}

#endif /* SRC_CORE_STD_ALTERNATIVES_STRING_HPP_ */
