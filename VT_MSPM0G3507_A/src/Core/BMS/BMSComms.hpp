/*
 * BMSSlaveInterface.hpp
 *
 *  Created on: Nov 20, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_BMS_BMSCOMMS_HPP_
#define SRC_CORE_BMS_BMSCOMMS_HPP_

#include "Core/common.h"
#include "Core/Networking/CAN.hpp"
#include "Middleware/BQ769x2/BQ76952.hpp"

/**
 * CAN bus packet content being sent between and from BMS devices.
 * this file should be the same for bms master and slave software
 * - datagram based network. not transaction based. makes life easier for everyone.
 * - BMS master-slave comms use can fd with 29b id, 20B max size. baud settings set in settings.hpp
 * - use explicit enum numbering
 */
namespace BMSComms {
    using namespace Networking::CAN;

    /** base error message priority, a "da pack is exploding" kind of situation */
    constexpr uint8_t BASE_PRI_ERROR    = 0b110;
    /** base internal message priority */
    constexpr uint8_t BASE_PRI_INT      = 0b100;
    /** base broad cast message priority */
    constexpr uint8_t BASE_PRI_BRD      = 0b010;
    /** Modbus translation packet priority */
    constexpr uint8_t PRI_MODBUS = getPriOffset(1, BASE_PRI_BRD);

    /** maximum transmission size between master and slave
     *  - limited by CAN ram settings (see settings.hpp).
     *      - tradeoff <rx fifo elements> vs <element size>
     */
    constexpr uint8_t MAX_PKT_SIZE_BYTES= 20;

    /** id's for 11b packets*/
    enum STD_ID : uint16_t {

    };

    /** id's for 29b packets. J1939 PDU-Format
     * - PF's >= 240 are broadcast
     * - ALL CHANGES HERE MUST BE UPDATED IN CAN ID FILTER (see system.hpp)
     */
    enum J1939_PF : uint8_t {
        MS      = 103,  // Master   -> Slave        (fifo) master ignores
        SM      = 104,  // Slave    -> Master       (fifo) slave ignores

        MOD     = 120,  // ModbusTCP translation    all listen, for modbus use ONLY

        B       = 247,  // general use broadcast    all listen
    };
    static_assert(J1939_PF::B >= 240); // J1939 standard, broadcasts >=240

    /** packet intended for Slave --> Master transmissions */
    enum PktSM_t : uint32_t {
        CELLV               = 0,    // cell voltage
        CELLT               = 1,    // cell temp

        // add entries & corresponding struct as needed
        // maximum message count limited by "type" size in PacketHeader
    };

    /** packet intended for Master --> Slave transmissions */
    enum PktMS_t : uint8_t {

    };

    /** packet intended for BMS --> <3rd party> transmissions */
    enum PktCanSt_t : uint8_t {

    };

    /** returns true if is a valid BMS packet id. does not check if intended for this device */
    bool isValidPacketID(DL_MCAN_RxBufElement const &);

    bool sendPacket(J1939_PF type, uint8_t J1939_JS, int8_t priorityOffset, void const * data, uint8_t len);

    uint8_t getID();
};

#endif /* SRC_CORE_BMS_BMSCOMMS_HPP_ */
