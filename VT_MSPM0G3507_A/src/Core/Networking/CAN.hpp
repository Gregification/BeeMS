/*
 * CANcomm.hpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#ifndef SRC_CORE_NETWORKING_CAN_HPP_
#define SRC_CORE_NETWORKING_CAN_HPP_

#include <stdint.h>

namespace Networking {

    /** Controller Area Network */
    namespace CAN {

        /** targeted for the J1939 standard
         *
         * ISO 11783-11 PGN tables down loads : https://www.isobus.net/isobus/
         * J1939 SPN table sources : https://felixequipment.com/Documents/Suspect-Parameter-Numbers-SPN-Codes.pdf
         * J1939 fault code look-up, Source Address, PGN, SPN, and FMI : https://lnx.numeralkod.com/wordpress/docs/errors-index/suspect-parameter-numbers-spn/
         */
        namespace J1939 {
            constexpr uint16_t BAD_SPN = 0;

            /** J1939 standard 29 bit packet ID
             * lower 29 bits used, the top 4 bits ignored
             *
             * image of bit layout : https://www.csselectronics.com/cdn/shop/files/j1939-pgn-18-bit-extended-can-identifier-pdu.svg
             */
            union __attribute__((__packed__)) ID {
                struct __attribute__((__packed__)) {
                    uint8_t     src_addr;
                    uint8_t     pdu_specific;
                    uint8_t     pdu_format;
                    uint8_t     data_page       : 1;
                    uint8_t                     : 1; // reserved by protocol
                    uint8_t     priority        : 3;
                };
                uint32_t raw : 29;
            };
            static_assert(sizeof(ID) == sizeof(uint32_t), "a J1939 ID is 29 bits");
        }

    }
}



#endif /* SRC_CORE_NETWORKING_CAN_HPP_ */
