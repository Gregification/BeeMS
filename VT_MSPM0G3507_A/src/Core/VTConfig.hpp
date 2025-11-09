/*
 * VTConfig.hpp
 *
 *  Created on: Nov 8, 2025
 *      Author: turtl
 * 
 * configuration options for a Voltage Tap board
 */

#ifndef SRC_CORE_VTCONFIG_HPP_
#define SRC_CORE_VTCONFIG_HPP_

#include "common.h" 

namespace VTCONFIG {
    /**
     * BQ76952 configuration.
     * this is no where near all the options on the BQ. 
     * Add options as you need them
     * BQTRM: https://www.ti.com/lit/ug/sluuby2b/sluuby2b.pdf
     */
    struct __attribute__((packed)) BQ76952SSetting {
        struct __attribute__((packed)) {
            /* min fuse blow voltage (units of 10mV)
             * reg:BQTM.13.3.1.1/135 . desc:BQTRM.6.8/61 */
            uint16_t minBlowFuseVoltage_10mV;

            /* fuse blow timeout (seconds)
             * will assert fuse blow for X time
             * BATRM.13.3.1.2/135 */
            uint8_t timeout;
        } Fuse;

        struct __attribute__((packed)) {
            
            /* BATRM.13.3.2.1/135 */
            union {
                uint16_t Raw;
                struct {
                    unsigned int wake_speed     : 2;
                    unsigned int loop_slow      : 2;
                    unsigned int cb_loop_slow   : 2;
                    unsigned int fastadc        : 1;
                    unsigned int otsd           : 1;
                    unsigned int sleep          : 1;
                    unsigned int dpslp_lfo      : 1;
                    unsigned int dpslp_ldo      : 1;
                    unsigned int dpslp_pd       : 1;
                    unsigned int shut_ts2       : 1;
                    unsigned int dpslp_ot       : 1;
                    unsigned int                : 2; // reserved
                };
            } powerConfig;
            static_assert(sizeof(powerConfig) == sizeof(uint16_t));

        } Config;
    };  
}


#endif /* SRC_CORE_VTCONFIG_HPP_ */
