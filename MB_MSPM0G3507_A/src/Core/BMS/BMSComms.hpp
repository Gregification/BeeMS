/*
 * BMSSlaveInterface.hpp
 *
 *  Created on: Nov 20, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_BMS_BMSCOMMS_HPP_
#define SRC_CORE_BMS_BMSCOMMS_HPP_

#include "Core/common.h"

/**
 * packet content being sent between master and slave
 * - everything is single packet, no splitting data across packets. makes life easier for everyone.
 */
namespace BMSComms {
    // maximum transmission size between master and slave
    constexpr buffersize_t MAX_MSSM_PACKET_SIZE_B = 64; // 64 Bytes
    // the mspm0g3507, intended MCU for this project, can use CAN-FD. which goes in 12/16/20/24/32/48/64 B
    constexpr bool isValidPacketSize(int N) { return N<=8 || N == 12 || N == 16 || N == 20 || N == 24 || N == 32 || N == 48 || N== 64; }

    /** packet for Slave --> Master transmissions */
    enum PktTypeSM_t : uint8_t {
        CELLV               = 0,    // cell voltage
        DELTA_CC            = 1,    // coulomb counting change
        TOTAL_CC            = 2,    // slave board CC
        // add entries & corresponding struct as needed
        // maximum message count limited by "type" size in PacketHeader
    };

    enum PktTypeMS_t : uint8_t {
    };

    struct __attribute__((packed)) PacketHeader {
        union __attribute__((packed)) {
            struct __attribute__((packed)) {
                PktTypeSM_t typeSM : 4;
                uint8_t slaveID    : 4;
            };
            struct __attribute__((packed)) {
                PktTypeMS_t typeMS: 4;
            };
        };
        uint8_t data[0];
    };

    struct __attribute__((packed)) Pkt_CellV {
        uint8_t baseCellNum;
        uint8_t cellCount;
        int16_t cellsmV[14];
        uint8_t                 : 8; // reserved
    };
    static_assert(isValidPacketSize((sizeof(PacketHeader) + sizeof(Pkt_CellV))));

    struct __attribute__((packed)) Pkt_DeltaCC {
        int64_t accumulatedmC   : 40;
        uint32_t timeddS        : 16;
    };
    static_assert(isValidPacketSize((sizeof(PacketHeader) + sizeof(Pkt_DeltaCC))));

    struct __attribute__((packed)) Pkt_TotalCC {
        int64_t accumulatedmC   : 56; // units of 0.001C
    };
    static_assert(isValidPacketSize((sizeof(PacketHeader) + sizeof(Pkt_TotalCC))));

};

#endif /* SRC_CORE_BMS_BMSCOMMS_HPP_ */
