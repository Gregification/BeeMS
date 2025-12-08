/*
 * BMSCommon.hpp
 *
 *  Created on: Nov 20, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_BMS_BMSCOMMON_HPP_
#define SRC_CORE_BMS_BMSCOMMON_HPP_

#include "Core/common.h"

struct __attribute__((packed)) TempSensor {
    /** describes what the temperature measurement is of.
     *  different safety ranges for different hardware */
    enum TSTarget {
        CELL,           // battery cell itself, abides by cell limits
        MID_RANGE,      // considered in safety, abides by mid-range limits
        HIGH_RANGE,     // considered in safety, abides by high-range limits
        UNCHECKED,      // not considered in safety
    };

    TSTarget type       : 3;
    unsigned int degdC  : 13;   // degrees in 0.1C
};
static_assert(sizeof(TempSensor) == sizeof(uint16_t));


#endif /* SRC_CORE_BMS_BMSCOMMON_HPP_ */
