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

namespace VTCONFIG
{
/**
 * BQ76952 configuration.
 * this is no where near all the options on the BQ.
 * Add options as you need them
 * BQTRM: https://www.ti.com/lit/ug/sluuby2b/sluuby2b.pdf
 */
struct __attribute__((packed)) BQ76952SSetting
{
    struct __attribute__((packed))
    {
        /* min fuse blow voltage (units of 10mV)
         * reg:BQTM.13.3.1.1/135 . desc:BQTRM.6.8/61 */
        uint16_t minBlowFuseVoltage_10mV;

        /* fuse blow timeout (seconds)
         * will assert fuse blow for X time
         * BATRM.13.3.1.2/135 */
        uint8_t timeout;
    } Fuse;

    struct __attribute__((packed))
    {

        /* BATRM.13.3.2.1/135 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int wake_speed :2;
                unsigned int loop_slow :2;
                unsigned int cb_loop_slow :2;
                unsigned int fastadc :1;
                unsigned int otsd :1;
                unsigned int sleep :1;
                unsigned int dpslp_lfo :1;
                unsigned int dpslp_ldo :1;
                unsigned int dpslp_pd :1;
                unsigned int shut_ts2 :1;
                unsigned int dpslp_ot :1;
                unsigned int :2; // reserved
            };
        } powerConfig;
        static_assert(sizeof(powerConfig) == sizeof(uint16_t));

        /* BATRM.13.3.2.2/137 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int enable1 :1;
                unsigned int reg1_v :3;
                unsigned int enable2 :1;
                unsigned int reg2_v :3;
            };
        } REG12Config;
        static_assert(sizeof(REG12Config) == sizeof(uint16_t));

        /* BATRM.13.3.2.3/138 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int enable0 :1;
                unsigned int :7; // reserved
            };
        } REG0Config;
        static_assert(sizeof(REG0Config) == sizeof(uint16_t));

        /* BATRM.13.3.2.4/138 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int toggle_time :4;
                unsigned int toggle_opt :2;
                unsigned int :2; // reserved
            };
        } hwdOptions;
        static_assert(sizeof(hwdOptions) == sizeof(uint16_t));

        /* BATRM.13.3.2.5/138 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int mode :5; // 5 bits for mode. not sure what to put here since its not a normal register
                unsigned int :3; // reserved
            };
        } commType;
        static_assert(sizeof(commType) == sizeof(uint16_t));

        /* BATRM.13.3.2.6/139 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int addr :8;
            };
        } i2cADDR;
        static_assert(sizeof(i2cADDR) == sizeof(uint16_t));

        /* BATRM.13.3.2.7/139 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int :5; // reserved
                unsigned int filt :1;
                unsigned int miso_reg1 :1;
                unsigned int :1; // reserved
            };
        } spiConfig;
        static_assert(sizeof(spiConfig) == sizeof(uint16_t));

        /* BATRM.13.3.2.8/139 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int time :8;
            };
        } commIdleTime;
        static_assert(sizeof(commIdleTime) == sizeof(uint16_t));

        /* BATRM.13.3.2.9/140 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int function :2;
                unsigned int option :6;
            };
        } cfetoffConfig;
        static_assert(sizeof(cfetoffConfig) == sizeof(uint16_t));

        /* BATRM.13.3.2.10/142 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int function :2;
                unsigned int option :6;
            };
        } dfetoffConfig;
        static_assert(sizeof(dfetoffConfig) == sizeof(uint16_t));

        /* BATRM.13.3.2.11/144 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int function :2;
                unsigned int option :6;
            };
        } alertConfig;
        static_assert(sizeof(alertConfig) == sizeof(uint16_t));

        /* BATRM.13.3.2.12/146 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int function :2;
                unsigned int option :6;
            };
        } TS1Config;
        static_assert(sizeof(TS1Config) == sizeof(uint16_t));

        /* BATRM.13.3.2.13/146 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int function :2;
                unsigned int option :6;
            };
        } TS2Config;
        static_assert(sizeof(TS2Config) == sizeof(uint16_t));

        /* BATRM.13.3.2.14/147 */
        union
        {
            uint16_t Raw;
            struct
            {
                unsigned int function :2;
                unsigned int option :6;
            };
        } TS3Config;
        static_assert(sizeof(TS3Config) == sizeof(uint16_t));

    } Config;
};
}

#endif /* SRC_CORE_VTCONFIG_HPP_ */
