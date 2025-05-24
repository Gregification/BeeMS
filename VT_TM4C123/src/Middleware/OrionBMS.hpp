/*
 * OrionBMS.hpp
 *
 *  Created on: May 23, 2025
 *      Author: FSAE
 */

#ifndef SRC_MIDDLEWARE_ORIONBMS_HPP_
#define SRC_MIDDLEWARE_ORIONBMS_HPP_

#include <stdint.h>

namespace OrionBMS {
    /* Thermistor Expansion Module : https://www.orionbms.com/products/thermistor-expansion-module/ */
    namespace TEM {

        /* CAN protocol information : https://www.orionbms.com/downloads/misc/thermistor_module_canbus.pdf */
        namespace CAN {

            /* Thermistor General Broadcast */
            struct TGB {
                constexpr static uint32_t can_id = 0x1838F380;

                struct Packet {
                    uint16_t    id_abs;   // thermistor ID relative to all configured thermistor modules
                    int8_t      val;      // (deg C) thermistor value
                    uint8_t     id_rel;   // thermistor ID relative to this module
                    int8_t      lowest_val; // (deg C) lowest thermister value of the thermistor module
                    int8_t      highest_val;// (deg C) highest thermistor value of the thermistor module
                    uint8_t     highest_id; // (zero indexed) highest thermistor ID on the module
                    uint8_t     lowest_id;  // (zero indexed) lowest thermistor ID on the module
                };
                static_assert(sizeof(Packet) == 8, "incorrect length of can packet");
                Packet data;

            };

            /* Thermistor Module General Broadcase */
            struct TMGB {
                constexpr static uint32_t can_id = 0x1839F380;

                struct Packet {
                    uint8_t     module_number; // thermistor module number
                    int8_t      lowest_val; // (deg C) lowest thermister value of the thermistor module
                    int8_t      highest_val;// (deg C) highest thermistor value of the thermistor module
                    int8_t      average_val;// (deg C) average thermistor value of the thermistor module
                    uint8_t     therms_enabled_count; // number of thermistors enabled; bit 0x80 indicates a fault is present
                    uint8_t     highest_id; // (zero indexed) highest thermistor ID on the module
                    uint8_t     lowest_id;  // (zero indexed) lowest thermistor ID on the module
                    uint8_t     checksum;   // (sum of all bytes + 0x39 + length)

                    inline bool hasThermCountFault() const { return therms_enabled_count == 0x80; }
                    void updateChecksum();
                };
                static_assert(sizeof(Packet) == 8, "incorrect length of can packet");
                Packet data;
            };

        }

    }
}

#endif /* SRC_MIDDLEWARE_ORIONBMS_HPP_ */
