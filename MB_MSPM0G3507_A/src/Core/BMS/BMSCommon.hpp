/*
 * BMSCommon.hpp
 *
 *  Created on: Nov 20, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_BMS_BMSCOMMON_HPP_
#define SRC_CORE_BMS_BMSCOMMON_HPP_

#include "Core/common.h"

namespace BMSCommon {
    typedef uint32_t SafteyStatus_t;

    /* maximum number IC's per "module".
     */
    constexpr uint8_t MAX_ICsPerModule  = 2;
};

#endif /* SRC_CORE_BMS_BMSCOMMON_HPP_ */
