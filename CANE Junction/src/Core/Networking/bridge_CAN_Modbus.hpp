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

            struct Meta_t {
                uint8_t socket;
                uint8_t initiatorID;
            };

            /** populates CAN data buffer from a ModbusTCP packet and sets the length. nothing else is set.
             * - CAN ID j1939::pdu is modified
             * - "in" is assumed to be in network byte format
             * returns true if success. only reason it would fail is if packet to large */
            bool ModbusTCP_to_CAN(Modbus::MBAPHeader const * in, DL_MCAN_TxBufElement * out, Meta_t const * meta);

            /** translates CAN to ModbusTCP. returns true if success. */
            bool CAN_to_ModbusTCP(DL_MCAN_RxBufElement const * in, Modbus::MBAPHeader * out, Meta_t * meta);

            union __attribute__((__packed__)) CANPacket {
                uint8_t raw[DL_MCAN_MAX_PAYLOAD_BYTES];

                struct __attribute__((__packed__)) _Header {
                    uint8_t     ipsocketnum;
                    uint16_t    transactionID;  // Modbus transaction id
                    uint8_t     aduDataLen;
                    Modbus::ADUPacket adu;
                } header;
            };
        }
    }
}



#endif /* SRC_CORE_NETWORKING_BRIDGE_CAN_MODBUS_HPP_ */
