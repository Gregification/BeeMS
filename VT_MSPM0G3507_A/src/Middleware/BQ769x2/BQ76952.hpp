/*
 * BQ76952.hpp
 *
 *  Created on: Jul 20, 2025
 *      Author: FSAE
 */

#ifndef SRC_MIDDLEWARE_BQ76952_HPP_
#define SRC_MIDDLEWARE_BQ76952_HPP_

#include <stdint.h>

#include "Core/system.hpp"

#include "BQ769x2_PROTOCOL.hpp"


/*
 * a wrapper Class over the C style protocol library
 */
class BQ76952 {
public:
    System::SPI::SPI * const spi;
    System::GPIO::GPIO const * const cs;

    union BQ76952PinConfig {
        uint8_t Raw;
        struct __attribute__((packed)) {
            unsigned int function   : 2; // function
            unsigned int opt1_0     : 2;
            unsigned int opt3_2     : 2;
            unsigned int opt5_4     : 2;
        };
        // any other bit field compile time combo has to be done manually though Raw

        bool operator==(const BQ76952PinConfig& other) const;
    };
    static_assert(sizeof(BQ76952PinConfig) == sizeof(uint8_t));

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
            uint8_t timeout_S;
        } Fuse;

        struct __attribute__((packed)) {

            /* BATRM.13.3.2.1/135 */
            union {
                uint16_t Raw;
                struct __attribute__((packed)) {
                    unsigned int wake_speed :2;     // CCrate during SLEEP for wake trigger
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
            // should be setup though OTPM at factory or in-house when assembling board
            /* WARNING: settings for BQ PSU output. can damage any attached hardware.
             *      double check schematic before fiddling with
             */
//            union {
//                uint8_t Raw;
//                struct __attribute__((packed)) {
//                    unsigned int enable1 :1;
//                    unsigned int reg1_v :3;
//                    unsigned int enable2 :1;
//                    unsigned int reg2_v :3;
//                };
//            } REG12Config;
//            static_assert(sizeof(REG12Config) == sizeof(uint8_t));

            /* BATRM.13.3.2.3/138 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int enable0 :1;
                    unsigned int :7; // reserved
                };
            } REG0Config;
            static_assert(sizeof(REG0Config) == sizeof(uint8_t));

            /* BATRM.13.3.2.4/138 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int toggle_time :4;
                    unsigned int toggle_opt :2;
                    unsigned int :2; // reserved
                };
            } HWDRegulatorOptions;
            static_assert(sizeof(HWDRegulatorOptions) == sizeof(uint8_t));

            /* BATRM.13.3.2.5/138 */
            // no touch. should be configured though OTP
            // - the person who designed the hardware should take care of it
//            uint8_t commType;

            /* BATRM.13.3.2.6/139 */
            // ah hel no. never use this crap on the MSPM0G3507.
    //        union {
    //            uint16_t Raw;
    //            struct __attribute__((packed)) {
    //                unsigned int addr :8;
    //            };
    //        } i2cADDR;
    //        static_assert(sizeof(i2cADDR) == sizeof(uint16_t));

            /* BATRM.13.3.2.7/139 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int :5; // reserved
                    unsigned int filt :1;
                    unsigned int miso_reg1 :1;
                    unsigned int :1; // reserved
                };
            } spiConfig;
            static_assert(sizeof(spiConfig) == sizeof(uint8_t));

            /* BATRM.13.3.2.8/139 */
            uint8_t commIdleTime_S;

            /* BATRM.13.3.2.9/140 */
            BQ76952PinConfig cfetoffPinConfig;

            /* BATRM.13.3.2.10/142 */
            BQ76952PinConfig dfetoffPinConfig;

            /* BATRM.13.3.2.11/144 */
            BQ76952PinConfig alertPinConfig;

            /* BATRM.13.3.2.12/146 */
            BQ76952PinConfig TS1Config;

            /* BATRM.13.3.2.13/146 */
            BQ76952PinConfig TS2Config;

            /* BATRM.13.3.2.14/147 */
            BQ76952PinConfig TS3Config;

            /* BATRM.13.3.2.15/148 */
            BQ76952PinConfig HDQPinConfig;

            /* BATRM.13.3.2.16/150 */
            BQ76952PinConfig DCHGPinConfig;

            /* BATRM.13.3.2.17/152 */
            BQ76952PinConfig DDSGPinConfig;

            /* BATRM.13.3.2.18/154 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int user_amps  : 2;
                    unsigned int user_volts : 1;
                    unsigned int tint_en    : 1;
                    unsigned int tint_fett  : 1;
                    unsigned int            : 3; // reserved
                };
            } DAConfig;
            static_assert(sizeof(DAConfig) == sizeof(uint8_t));

            /* BATRM.13.3.2.19/154
             * bit array of what cells are enabled. BV(X) -> cell between taps (x) and (x+1)*/
            uint16_t VcellMode;

            /* BATRM.13.3.2.20/155 */
            uint8_t CC3Samples;

        } Configuration;

        bool operator==(const BQ76952SSetting& other) const;
    };

    struct DAStatus5 {
        uint16_t Vreg18;          // Bytes 0-1: VREG18, 16-bit ADC counts
        uint16_t VSS;             // Bytes 2-3: VSS, 16-bit ADC counts
        uint16_t MaxCellVoltage;  // Bytes 4-5: Max Cell Voltage, mV
        uint16_t MinCellVoltage;  // Bytes 6-7: Min Cell Voltage, mV
        uint16_t BatteryVoltageSum; // Bytes 8-9: Battery Voltage Sum, cV
        uint16_t AvgCellTemperature; // Bytes 10-11: Avg Cell Temperature, 0.1 K
        uint16_t FETTemperature;  // Bytes 12-13: FET Temperature, 0.1 K
        uint16_t MaxCellTemperature; // Bytes 14-15: Max Cell Temperature, 0.1 K
        uint16_t MinCellTemperature; // Bytes 16-17: Min Cell Temperature, 0.1 K
        uint16_t SecondAvgCellTemperature; // Bytes 18-19: Avg Cell Temperature, 0.1 K (renamed to avoid conflict)
        uint16_t CC3Current;      // Bytes 20-21: CC3 Current, userA
        uint16_t CC1Current;      // Bytes 22-23: CC1 Current, userA
        uint32_t CC2Counts;       // Bytes 24-27: CC2 Counts, 32-bit ADC counts
        uint32_t CC3Counts;       // Bytes 28-31: CC3 Counts, 32-bit ADC counts
    };

    bool getConfig(BQ76952SSetting *);
    bool setConfig(BQ76952SSetting const *);

    bool unseal(uint32_t key);

    bool sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd cmd);

    bool sendSubcommandR(BQ769X2_PROTOCOL::Cmd cmd, void * data_out, uint8_t datalen);
    bool sendSubcommandW(BQ769X2_PROTOCOL::Cmd cmd, uint16_t data, uint8_t datalen);

    bool sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt command, void * data_out, uint8_t datalen);
    bool sendDirectCommandW(BQ769X2_PROTOCOL::CmdDrt command, void * data, uint8_t datalen);

    bool sendCommandR(BQ769X2_PROTOCOL::Cmd command, void * data_out, uint8_t datalen);

    /* writes a 16b register of data memory
     * - see bqTM.13.9/198 for register lengths
     */
    bool setRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, uint16_t reg_data, uint8_t datalen);

    /* reads a 16b register of data memory
     * - bqTM.13.9/198
     * returns number of bytes read
     */
    uint8_t getRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, void * reg_data_out, uint8_t datalen);
};


#endif /* SRC_MIDDLEWARE_BQ76952_HPP_ */
