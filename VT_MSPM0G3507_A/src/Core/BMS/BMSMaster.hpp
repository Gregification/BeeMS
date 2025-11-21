/*
 * BMSMaster.hpp
 *
 *  Created on: Nov 20, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_BMSMASTER_HPP_
#define SRC_CORE_BMSMASTER_HPP_

#include "BMSCommon.hpp"
#include "BMSComms.hpp"
#include "FreeRTOS.h"

/**
 * the BMS master board logic,
 *  interface(CAN, Ethernet, etc) specific logic is abstracted
 *
 * - intended to run within a FreeRTOS task
 * - intended for LiPo cells
 * - code is basically just C, c++ only used for handy syntax. IDE dosent play nice with C++
 * - effort was made to steer clear of STL
 * - all allocations are compile time
 * - slave boards periodically transmit basic info
 * - master board will also request data from slave boards as needed
 */

template <uint8_t _MAX_SLAVES> // max slave boards
struct BMSMaster {
    static constexpr uint8_t MAX_SLAVES = _MAX_SLAVES;

    /**
     * information on remote slave boards
     */
    struct __attribute__((packed)) SlaveBoard {
        static constexpr uint8_t MAX_CELLS = 16;
        static constexpr uint8_t BAD_ID = 0;
        static constexpr uint8_t MAX_TEMP_SENSORS = MAX_CELLS;

        uint8_t ID;

        uint8_t cellCount;
        uint16_t cellsmV;

        uint64_t ccmC;                  // coulomb counter in mC units
        TickType_t ccRuntime;           // coulomb counter run time

        uint8_t tempSCount;
        TempSensor tempS[MAX_TEMP_SENSORS];

        TickType_t lastUpdate;
    };
    SlaveBoard slaves[MAX_SLAVES];

    /** resets the safety latch */
    void resetSafetyLatch();

    void processPacket(BMSComms::PacketHeader const * rx);
};

#endif /* SRC_CORE_BMSMASTER_HPP_ */
