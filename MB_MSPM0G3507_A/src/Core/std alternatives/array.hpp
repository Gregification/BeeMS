/*
 * array.hpp
 *
 *  Created on: Nov 20, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_STD_ALTERNATIVES_ARRAY_HPP_
#define SRC_CORE_STD_ALTERNATIVES_ARRAY_HPP_

#include "Core/common.h"

template <typename T, buffersize_t N>
class Array {
private:
    T data[N];

public:
    const T& operator[](buffersize_t index) const {
        return data[index];
    }

    T& operator[](buffersize_t index) {
        return data[index];
    }

    constexpr buffersize_t size(){
        return N;
    }
};


#endif /* SRC_CORE_STD_ALTERNATIVES_ARRAY_HPP_ */
