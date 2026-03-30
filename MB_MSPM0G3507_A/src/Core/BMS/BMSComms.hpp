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
#include "BMSCommon.hpp"

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
    constexpr uint8_t MAX_PKT_SIZE_BYTES = 20;

    /** id's for 11b packets*/
    enum STD_ID : uint16_t {

    };

    /** id's for 29b packets. J1939 PDU-Format
     * - PF's >= 240 are broadcast
     * - ALL CHANGES HERE MUST BE UPDATED IN CAN ID FILTER (see system.hpp)
     */
    enum J1939_PF_e : uint8_t {
        MS      = 103,  // Master   -> Slave        (fifo) master ignores
        SM      = 104,  // Slave    -> Master       (fifo) slave ignores

        MOD     = 120,  // ModbusTCP translation    all listen, for modbus use ONLY

        B       = 247,  // general use broadcast    all listen
    };
    static_assert(J1939_PF_e::B >= 240); // J1939 standard, broadcasts >=240

    /** packet intended for Slave --> Master transmissions */
    enum PktSM_JS_e : uint32_t {
        STATUS1             = 0,    // state of operations
        STATUS2             = 1,    // state of operations
        CELLV               = 5,    // cell voltage
        CELLT               = 6,    // cell temp

        // add entries & corresponding struct as needed
        // maximum message count limited by "type" size in PacketHeader
    };

    /** packet intended for Master --> Slave transmissions */
    enum PktMS_JS_e : uint8_t {
        MODB_REG                    = 0,    // modbus but packed custom so its smaller and faster, used for general r/w operaitons
        RESTART                     = 1,    // restarts slave device

        //TODO
//        LOAD_BUFFER_FROM_CAN        = 19,   // reads can packet into buffer
//        SAVE_PROFILE_TO_BUFF        = 20,   // slave writes settings to buffer
//        LOAD_PROFILE_FROM_BUFF      = 21,   // slave loads buffer contents as profile
//        SAVE_BUFFER_AS_PROFILE      = 22,   // writes buffer contents to profile non volatile memory
//        LOAD_PROFILE_NVM_TO_BUFF    = 23,   // loads profile from memory to buffer
    };


    /*** data packets: SM *********************************************/

    /** this packet is a keep alive, if not periodically sent BMS will fault */
    struct __attribute__((__packed__)) SM_STATUS1_t {
        bool error                  : 1;    // true if interlock is OK

        BMSCommon::SafteyStatus_t ICSafetyStatus[BMSCommon::Module::MAX_ICs];
        BMSCommon::packcV_t stack_cV;
        BMSCommon::Module::STATE_e state;
    };
    static_assert(sizeof(SM_STATUS1_t) <= MAX_PKT_SIZE_BYTES);

    struct __attribute__((__packed__)) SM_STATUS2_t {
        struct __attribute__((__packed__)) {
            BMSCommon::cDegC_t  min_dDegC;
            BMSCommon::cDegC_t  max_dDegC;
            BMSCommon::cDegC_t  avg_dDegC;
        } board;
        BMSCommon::cDegC_t  ambient;
    };
    static_assert(sizeof(SM_STATUS2_t) <= MAX_PKT_SIZE_BYTES);

    struct __attribute__((__packed__)) SM_CELLV_t {
        static constexpr int MAX_CELL_N = MAX_PKT_SIZE_BYTES/2 - 2;

        uint8_t base_cell;                   // starting cell. cell1+ would be #0
        unsigned int cellCount       : 6;
        unsigned int                 : 2;    // reserved
        BMSCommon::cellmV_t mV[MAX_CELL_N];
    };
    static_assert(sizeof(SM_CELLV_t) != MAX_PKT_SIZE_BYTES);
    static_assert(SM_CELLV_t::MAX_CELL_N < 10, "very small packet? something is wrong");

    struct __attribute__((__packed__)) SM_CELLT_t {
        static constexpr int MAX_CELL_N = MAX_PKT_SIZE_BYTES/2 - 2;

        uint8_t base_cell;                   // starting cell. cell1+ would be #0
        unsigned int cellCount       : 6;
        unsigned int                 : 2;    // reserved
        BMSCommon::cDegC_t dDegC[MAX_CELL_N];
    };
    static_assert(sizeof(SM_CELLT_t) != MAX_PKT_SIZE_BYTES);
    static_assert(SM_CELLT_t::MAX_CELL_N < 10, "very small packet? something is wrong");


    /*** data packets: MS *********************************************/

    struct __attribute__((__packed__)) MS_MODB_REG_t {
        static constexpr int MAX_DATA_16N = MAX_PKT_SIZE_BYTES/2 - 2;

        enum OP {
            R,W,
        };
        unsigned int operation      : 2;
        unsigned int data8len       : 6;
        uint16_t reg_addr;
        uint16_t data[MAX_DATA_16N];
    };
    static_assert(sizeof(MS_MODB_REG_t) <= MAX_PKT_SIZE_BYTES);


    /*** data packets: broadcasts *************************************/


    /******************************************************************/

    /** returns true if is a valid BMS packet id. does not check if intended for this device */
    bool isValidPacketID(DL_MCAN_RxBufElement const &);

    bool sendPacket(J1939_PF_e type, uint8_t J1939_JS, int8_t priorityOffset, void const * data, uint8_t len);

    uint8_t getID();
};

#endif /* SRC_CORE_BMS_BMSCOMMS_HPP_ */
