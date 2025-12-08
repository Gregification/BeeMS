/*
 * Modbus.hpp
 *
 *  Created on: Dec 2, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_NETWORKING_MODBUS_HPP_
#define SRC_CORE_NETWORKING_MODBUS_HPP_

#include <stdint.h>
#include <stdbool.h>

namespace Networking {

    /**
     * info & crc code from : https://www.modbustools.com/modbus.html
     *
     * note:
     *  - Modbus-TCP : is just a Modbus-RTU packet over TCP, except that it dosent have a CRC field
     */
    namespace Modbus {

        /*** common ******************************************/

        enum Function : uint8_t {
            R_COILS             = 1,
            R_DISRETE_INPUTS    = 2,
            R_HOLDING_REGS      = 3,
            R_INPUT_REGS        = 4,
            W_COIL              = 5,
            W_REG               = 6, // \-
            W_COILS             = 0x0F, // -/
            W_REGS              = 0x10,
        };


        /*** packet layout ***********************************/

        struct __attribute__((__packed__)) RTUPacket {
            uint8_t address;
            Function func;
            uint8_t data[0];
        };
        static_assert(sizeof(RTUPacket) == sizeof(uint16_t));

        // this is just another interpretation of the RTU packet but intended for the MBAT style header
        struct __attribute__((__packed__)) ADUPacket {
            Function func;
            uint8_t data[0];
        };
        static_assert(sizeof(ADUPacket) == sizeof(uint8_t));

        struct __attribute__((__packed__)) MBAPHeader {
            static constexpr uint16_t PROTOCOL_ID_MODBUS = 0; // always 0 for Modbus

            uint16_t transactionID;
            uint16_t protocolID = PROTOCOL_ID_MODBUS;
            uint16_t len;               // includes 'funciton code' and 'unit id'. (+2 bytes)
            uint8_t  unitID;            // 0x00 or 0xFF for Modbus-TCP/IP
            ADUPacket adu[0];
        };
        static_assert(sizeof(MBAPHeader) == 7);


        /*** function specific layouts ***********************/

        struct __attribute__((__packed__)) F_Holding_REQ {
            uint16_t start;
            uint16_t len;
        };
        static_assert(sizeof(F_Holding_REQ) == sizeof(uint32_t));

        struct __attribute__((__packed__)) F_Holding_RES {
            uint8_t byteCount;
            uint16_t values[0];
        };
        static_assert(sizeof(F_Holding_REQ) == sizeof(uint32_t));

        /*** functions ***************************************/

        uint16_t calcRTUCRC16(uint8_t const * raw, uint16_t len);
    }
}


#endif /* SRC_CORE_NETWORKING_MODBUS_HPP_ */
