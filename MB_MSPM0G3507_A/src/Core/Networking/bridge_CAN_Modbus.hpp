/*
 * bridge_CAN_Modbus.hpp
 *
 *  Created on: Dec 11, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_NETWORKING_BRIDGE_CAN_MODBUS_HPP_
#define SRC_CORE_NETWORKING_BRIDGE_CAN_MODBUS_HPP_

#include <stdint.h>
#include <ti/driverlib/driverlib.h>

#include "CAN.hpp"
#include "Modbus.hpp"

namespace Networking {
    namespace Bridge {
        namespace CANModbus {
            constexpr uint8_t PKTBUFFSIZE = DL_MCAN_MAX_PAYLOAD_BYTES + sizeof(CAN::J1939::ID);

            /** PDU-format value used to indicate Modbus-TCP translation */
            constexpr uint8_t J1939_PDU_FORMAT   = 0x67;

            /** PDU-specific value is just the Modbus::Funciton */

            /** populates CAN data buffer from a ModbusTCP packet and sets the length. nothing else is set.
             * - CAN ID j1939::pdu is modified
             * - "in" is assumed to be in network byte format
             * returns true if success. only reason it would fail is if packet to large */
            bool ModbusTCP_to_CAN(Modbus::MBAPHeader const * in, DL_MCAN_TxBufElement * out, uint8_t socket);

            /** translates CAN to ModbusTCP. returns true if success. */
            bool CAN_to_ModbusTCP(DL_MCAN_RxBufElement const * in, Modbus::MBAPHeader * out, uint8_t * socket);

            union __attribute__((__packed__)) CANPacket {
                uint8_t raw[DL_MCAN_MAX_PAYLOAD_BYTES];

                struct __attribute__((__packed__)) _Header {
                    uint8_t     ipsocketnum;
                    uint16_t    transactionID;  // Modbus transaction id
                    uint8_t     unitID;
                    uint8_t     mbatlen;        // byte count of mbat data
                    uint8_t     adudata[0];
                } header;
            };
        }
    }
}



#endif /* SRC_CORE_NETWORKING_BRIDGE_CAN_MODBUS_HPP_ */
