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
//            BQ76952PinConfig cfetoffPinConfig;
// I am getting an error when I do this, I think its because in BQ76952.cpp we are doing .raw, but in operator not doing raw
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int OPT5_OPT0 : 6;
                    unsigned int PIN_FXN1_PIN_FXN0 : 2;
                };
            } cfetoffPinConfig;

            static_assert(sizeof(cfetoffPinConfig) == sizeof(uint8_t));


            /* BATRM.13.3.2.10/142 */
//            BQ76952PinConfig dfetoffPinConfig;
            union {
                uint8_t Raw;
                struct __attribute__((packed)){
                    unsigned int OPT5_OPT0 : 6;
                    unsigned int PIN_FXN1_PIN_FXN0 : 2;
                };
            } dfetoffPinConfig;
            static_assert(sizeof(dfetoffPinConfig) == sizeof(uint8_t));


            /* BATRM.13.3.2.11/144 */
//            BQ76952PinConfig alertPinConfig;
            union {
                uint8_t Raw;
                struct __attribute__((packed)){
                    unsigned int OPT5_OPT0 : 6;
                    unsigned int PIN_FXN1_PIN_FXN0 : 2;
                } ;
            } alertPinConfig;
            static_assert(sizeof(alertPinConfig) == sizeof(uint8_t));

            /* BATRM.13.3.2.12/146 */
//            BQ76952PinConfig TS1Config;
            union {
                uint8_t Raw;
                struct __attribute__((packed)){
                    unsigned int OPT5_OPT0 : 6;
                    unsigned int PIN_FXN1_PIN_FXN0 : 2;
                } ;
            } TS1Config;
            static_assert(sizeof(TS1Config) == sizeof(uint8_t));

            /* BATRM.13.3.2.13/146 */
//            BQ76952PinConfig TS2Config;
            union {
                uint8_t Raw;
                struct __attribute__((packed)){
                    unsigned int OPT5_OPT0 : 6;
                    unsigned int PIN_FXN1_PIN_FXN0 : 2;
                } ;
            } TS2Config;
            static_assert(sizeof(TS2Config) == sizeof(uint8_t));

            /* BATRM.13.3.2.14/147 */
//            BQ76952PinConfig TS3Config;
            union {
                uint8_t Raw;
                struct __attribute__((packed)){
                    unsigned int OPT5_OPT0 : 6;
                    unsigned int PIN_FXN1_PIN_FXN0 : 2;
                };
            } TS3Config;
            static_assert(sizeof(TS3Config) == sizeof(uint8_t));

            /* BATRM.13.3.2.15/148 */
//            BQ76952PinConfig HDQPinConfig;
            union {
                uint8_t Raw;
                struct __attribute__((packed)){
                    unsigned int OPT5_OPT0 : 6;
                    unsigned int PIN_FXN1_PIN_FXN0 : 2;
                };
            } HDQPinConfig;
            static_assert(sizeof(HDQPinConfig) == sizeof(uint8_t));

            /* BATRM.13.3.2.16/150 */
//            BQ76952PinConfig DCHGPinConfig;
            union {
                uint8_t Raw;
                struct __attribute__((packed)){
                    unsigned int OPT5_OPT0 : 6;
                    unsigned int PIN_FXN1_PIN_FXN0 : 2;
                };
            } DCHGPinConfig;
            static_assert(sizeof(DCHGPinConfig) == sizeof(uint8_t));


            /* BATRM.13.3.2.17/152 */
//            BQ76952PinConfig DDSGPinConfig;
            union {
                uint8_t Raw;
                struct __attribute__((packed)){
                    unsigned int OPT5_OPT0 : 6;
                    unsigned int PIN_FXN1_PIN_FXN0 : 2;
                };
            } DDSGPinConfig;
            static_assert(sizeof(DDSGPinConfig) == sizeof(uint8_t));

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

        struct __attribute__((packed)) {
            /* BATRM.13.3.3.1/156 */
            union {
                uint16_t Raw;
                struct __attribute__((packed)) {
                    unsigned int : 1;                      // Bit 0: RSVD_0
                    unsigned int PF_FETS : 1;              // Bit 1: PF causes FETs off (Default: 1)
                    unsigned int PF_REGS : 1;              // Bit 2: PF causes regulators off (Default: 0)
                    unsigned int PF_DPSLP : 1;             // Bit 3: PF causes device to enter DEEPSLEEP (Default: 0)
                    unsigned int PF_FUSE : 1;              // Bit 4: PF causes fuse blow (Default: 0)
                    unsigned int PF_OTP : 1;               // Bit 5: PF status preserved across reset/written to OTP (Default: 0)
                    unsigned int : 1;                      // Bit 6: RSVD_0
                    unsigned int PACK_FUSE : 1;            // Bit 7: Use PACK voltage for Min Blow Fuse Voltage check (Default: 0)
                    unsigned int FETF_FUSE : 1;            // Bit 8: FETF bypasses Min Blow Fuse Voltage check (Default: 0)
                    unsigned int OCDL_CURR_RECOV : 1;      // Bit 9: OCD Latch recovers based on charge current (Default: 0)
                    unsigned int SCDL_CURR_RECOV : 1;      // Bit 10: SCD Latch recovers based on charge current (Default: 0)
                    unsigned int : 5;                      // Bits 11-15: RSVD_0 (5 total bits)
                };
            } protectionConfiguraiton;
            static_assert(sizeof(protectionConfiguraiton) == sizeof(uint16_t));

            /* BATRM.13.3.3.2/157 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int : 1;                      // Bit 0: RSVD_0
                    unsigned int : 1;                      // Bit 1: RSVD_0
                    unsigned int CUV : 1;                  // Bit 2: Cell Undervoltage Protection (Default: 0)
                    unsigned int COV : 1;                  // Bit 3: Cell Overvoltage Protection (Default: 1)
                    unsigned int OCC : 1;                  // Bit 4: Overcurrent in Charge Protection (Default: 0)
                    unsigned int OCD1 : 1;                 // Bit 5: Overcurrent in Discharge 1st Tier Protection (Default: 0)
                    unsigned int OCD2 : 1;                 // Bit 6: Overcurrent in Discharge 2nd Tier Protection (Default: 0)
                    unsigned int SCD : 1;                  // Bit 7: Short Circuit in Discharge Protection (Default: 1)
                };
            } enabledProtectionsA;
            static_assert(sizeof(enabledProtectionsA) == sizeof(uint8_t));

            /* BATRM.13.3.3.3/158 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int UTC : 1;                  // Bit 0: Undertemperature in Charge (Default: 0)
                    unsigned int UTD : 1;                  // Bit 1: Undertemperature in Discharge (Default: 0)
                    unsigned int UTINT : 1;                // Bit 2: Internal Undertemperature (Default: 0)
                    unsigned int : 1;                      // Bit 3: RSVD_0
                    unsigned int OTC : 1;                  // Bit 4: Overtemperature in Charge (Default: 0)
                    unsigned int OTD : 1;                  // Bit 5: Overtemperature in Discharge (Default: 0)
                    unsigned int OTINT : 1;                // Bit 6: Internal Overtemperature (Default: 0)
                    unsigned int OTF : 1;                  // Bit 7: FET Overtemperature (Default: 0)
                };
            } enabledProtectionsB;
            static_assert(sizeof(enabledProtectionsB) == sizeof(uint8_t));

            /* BATRM.13.3.3.4/159 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int : 1;                      // Bit 0: RSVD_0
                    unsigned int HWDF : 1;                 // Bit 1: Host Watchdog Fault (Default: 0)
                    unsigned int PTO : 1;                  // Bit 2: Precharge Timeout (Default: 0)
                    unsigned int : 1;                      // Bit 3: RSVD_0
                    unsigned int COVL : 1;                 // Bit 4: Cell Overvoltage Latch (Default: 0)
                    unsigned int OCDL : 1;                 // Bit 5: Overcurrent in Discharge Latch (Default: 0)
                    unsigned int SCDL : 1;                 // Bit 6: Short Circuit in Discharge Latch (Default: 0)
                    unsigned int OCD3 : 1;                 // Bit 7: Overcurrent in Discharge 3rd Tier Protection (Default: 0)
                };
            } enabledProtectionsC;
            static_assert(sizeof(enabledProtectionsC) == sizeof(uint8_t));

            /* BATRM.13.3.3.5/160 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int : 3;                      // Bits 0-2: RSVD_0
                    unsigned int COV : 1;                  // Bit 3: Cell Overvoltage Protection (Default: 1)
                    unsigned int OCC : 1;                  // Bit 4: Overcurrent in Charge Protection (Default: 1)
                    unsigned int : 2;                      // Bits 5-6: RSVD_0
                    unsigned int SCD : 1;                  // Bit 7: Short Circuit in Discharge Protection (Default: 1)
                };
            } chgFetProtectionsA;
            static_assert(sizeof(chgFetProtectionsA) == sizeof(uint8_t));

            /* BATRM.13.3.3.6/160 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int UTC : 1 = 1;                  // Bit 0: Undertemperature in Charge
                    unsigned int : 1;                          // Bit 1: RSVD_0
                    unsigned int UTINT : 1 = 1;                // Bit 2: Internal Undertemperature
                    unsigned int : 1;                          // Bit 3: RSVD_0
                    unsigned int OTC : 1 = 1;                  // Bit 4: Overtemperature in Charge
                    unsigned int : 1;                          // Bit 5: RSVD_0
                    unsigned int OTINT : 1 = 1;                // Bit 6: Internal Overtemperature
                    unsigned int OTF : 1 = 1;                  // Bit 7: FET Overtemperature
                };
            } chgFetProtectionsB;
            static_assert(sizeof(chgFetProtectionsB) == sizeof(uint8_t));

            /* BATRM.13.3.3.7/160 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int : 1;                          // Bit 0: RSVD_0
                    unsigned int HWDF : 1 = 1;                 // Bit 1: Host Watchdog Fault
                    unsigned int PTO : 1 = 1;                  // Bit 2: Precharge Timeout
                    unsigned int : 1;                          // Bit 3: RSVD_0
                    unsigned int COVL : 1 = 1;                 // Bit 4: Cell Overvoltage Latch
                    unsigned int : 1;                          // Bit 5: RSVD_0
                    unsigned int SCDL : 1 = 1;                 // Bit 6: Short Circuit in Discharge Latch
                    unsigned int : 1;                          // Bit 7: RSVD_0
                };
            } chgFetProtectionsC;
            static_assert(sizeof(chgFetProtectionsC) == sizeof(uint8_t));

            /* BATRM.13.3.3.8/161 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int : 2;                          // Bits 0-1: RSVD_0
                    unsigned int CUV : 1 = 1;                  // Bit 2: Cell Undervoltage Protection
                    unsigned int : 2;                          // Bits 3-4: RSVD_0
                    unsigned int OCD1 : 1 = 1;                 // Bit 5: Overcurrent in Discharge 1st Tier Protection
                    unsigned int OCD2 : 1 = 1;                 // Bit 6: Overcurrent in Discharge 2nd Tier Protection
                    unsigned int SCD : 1 = 1;                  // Bit 7: Short Circuit in Discharge Protection
                };
            } dsgFetProtectionsA;
            static_assert(sizeof(dsgFetProtectionsA) == sizeof(uint8_t));

            /* BATRM.13.3.3.9/161 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int : 1;                      // Bit 0: RSVD_0
                    unsigned int UTD : 1 = 1;              // Bit 1: Undertemperature in Discharge
                    unsigned int UTINT : 1 = 1;            // Bit 2: Internal Undertemperature
                    unsigned int : 2;                      // Bits 3-4: RSVD_0
                    unsigned int OTD : 1 = 1;              // Bit 5: Overtemperature in Discharge
                    unsigned int OTINT : 1 = 1;            // Bit 6: Internal Overtemperature
                    unsigned int OTF : 1 = 1;              // Bit 7: FET Overtemperature
                };
            } dsgFetProtectionsB;
            static_assert(sizeof(dsgFetProtectionsB) == sizeof(uint8_t));

            /* BATRM.13.3.3.10/162 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int : 1;                      // Bit 0: RSVD_0
                    unsigned int HWDF : 1 = 1;             // Bit 1: Host Watchdog Fault
                    unsigned int : 3;                      // Bits 2-4: RSVD_0
                    unsigned int OCDL : 1 = 1;             // Bit 5: Overcurrent in Discharge Latch
                    unsigned int SCDL : 1 = 1;             // Bit 6: Short Circuit in Discharge Latch
                    unsigned int OCD3 : 1 = 1;             // Bit 7: Overcurrent in Discharge 3rd Tier Protection
                };
            } dsgFetProtectionsC;
            static_assert(sizeof(dsgFetProtectionsC) == sizeof(uint8_t));

            /* BATRM.13.3.3.11/163 */
            uint16_t bodyDiodeThreshold;
        } Protection;

        struct __attribute__((packed)) {

            /* BATRM.13.3.4.1/164 */
            union {
                uint16_t Raw;
                struct __attribute__((packed)) {
                    unsigned int WAKE : 1 = 0;             // Bit 0: Device wakened from SLEEP mode
                    unsigned int ADSCAN : 1 = 0;           // Bit 1: Voltage ADC Scan Complete
                    unsigned int CB : 1 = 0;               // Bit 2: Cell balancing is active
                    unsigned int FUSE : 1 = 0;             // Bit 3: FUSE Pin Driven
                    unsigned int SHUTV : 1 = 0;            // Bit 4: Stack voltage below Shutdown Stack Voltage
                    unsigned int XDSG : 1 = 0;             // Bit 5: DSG FET is off
                    unsigned int XCHG : 1 = 0;             // Bit 6: CHG FET is off
                    unsigned int FULLSCAN : 1 = 0;         // Bit 7: Full Voltage Scan Complete
                    unsigned int : 1;                      // Bit 8: RSVD_0
                    unsigned int INITCOMP : 1 = 0;         // Bit 9: Initialization completed
                    unsigned int INITSTART : 1 = 0;        // Bit 10: Initialization started
                    unsigned int MSK_PFALERT : 1 = 1;      // Bit 11: Permanent Fail alert triggered and masked
                    unsigned int MSK_SFALERT : 1 = 1;      // Bit 12: Safety alert triggered and masked
                    unsigned int PF : 1 = 1;               // Bit 13: Enabled Permanent Fail fault triggers
                    unsigned int SSA : 1 = 1;              // Bit 14: Bit is set in Safety Status A
                    unsigned int SSBC : 1 = 1;             // Bit 15: Bit is set in Safety Status B/C
                };
            } defaultAlarmMask;
            static_assert(sizeof(defaultAlarmMask) == sizeof(uint16_t));

            /* BATRM.13.3.4.2/164 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int : 2;                      // Bits 0-1: RSVD_0
                    unsigned int CUV : 1 = 1;              // Bit 2: Cell Undervoltage Protection
                    unsigned int COV : 1 = 1;              // Bit 3: Cell Overvoltage Protection
                    unsigned int OCC : 1 = 1;              // Bit 4: Overcurrent in Charge Protection
                    unsigned int OCD1 : 1 = 1;             // Bit 5: Overcurrent in Discharge 1st Tier Protection
                    unsigned int OCD2 : 1 = 1;             // Bit 6: Overcurrent in Discharge 2nd Tier Protection
                    unsigned int SCD : 1 = 1;              // Bit 7: Short Circuit in Discharge Protection
                };
            } sfAlertMaskA;
            static_assert(sizeof(sfAlertMaskA) == sizeof(uint8_t));

            /* BATRM.13.3.4.3/164 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int UTC : 1 = 1;              // Bit 0: Undertemperature in Charge
                    unsigned int UTD : 1 = 1;              // Bit 1: Undertemperature in Discharge
                    unsigned int UTINT : 1 = 1;            // Bit 2: Internal Undertemperature
                    unsigned int : 1;                      // Bit 3: RSVD_0
                    unsigned int OTC : 1 = 1;              // Bit 4: Overtemperature in Charge
                    unsigned int OTD : 1 = 1;              // Bit 5: Overtemperature in Discharge
                    unsigned int OTINT : 1 = 1;            // Bit 6: Internal Overtemperature
                    unsigned int OTF : 1 = 1;              // Bit 7: FET Overtemperature
                };
            } sfAlertMaskB;
            static_assert(sizeof(sfAlertMaskB) == sizeof(uint8_t));

            /* BATRM.13.3.4.4/164 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int : 2;                      // Bits 0-1: RSVD_0/1
                    unsigned int PTO : 1 = 1;              // Bit 2: Precharge Timeout
                    unsigned int : 1;                      // Bit 3: RSVD_0
                    unsigned int COVL : 1 = 1;             // Bit 4: Cell Overvoltage Latch
                    unsigned int OCDL : 1 = 1;             // Bit 5: Overcurrent in Discharge Latch
                    unsigned int SCDL : 1 = 1;             // Bit 6: Short Circuit in Discharge Latch
                    unsigned int OCD3 : 1 = 1;             // Bit 7: Overcurrent in Discharge 3rd Tier Protection
                };
            } sfAlertMaskC;
            static_assert(sizeof(sfAlertMaskC) == sizeof(uint8_t));

            /* BATRM.13.3.4.5/ */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int SUV : 1 = 1;              // Bit 0: Safety Undervoltage
                    unsigned int SOV : 1 = 1;              // Bit 1: Safety Overvoltage
                    unsigned int SOCC : 1 = 1;             // Bit 2: Safety Overcurrent in Charge
                    unsigned int SOCD : 1 = 1;             // Bit 3: Safety Overcurrent in Discharge
                    unsigned int SOT : 1 = 1;              // Bit 4: Safety Overtemperature
                    unsigned int : 1;                      // Bit 5: RSVD_0
                    unsigned int SOTF : 1 = 1;             // Bit 6: Safety Over-FET-Temperature
                    unsigned int CUDEP : 1 = 0;            // Bit 7: Cell Undervoltage Depletion
                };
            } pfAlertMaskA;
            static_assert(sizeof(pfAlertMaskA) == sizeof(uint8_t));

            /* BATRM.13.3.4.6/164 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int CFETF : 1 = 1;            // Bit 0: CHG FET Fault
                    unsigned int DFETF : 1 = 1;            // Bit 1: DSG FET Fault
                    unsigned int TWO_LVL : 1 = 1;          // Bit 2: 2nd Level Protection (2LVL)
                    unsigned int VIMR : 1 = 1;             // Bit 3: Voltage Imbalance Measurement Range
                    unsigned int VIMA : 1 = 1;             // Bit 4: Voltage Imbalance Measurement Area
                    unsigned int : 2;                      // Bits 5-6: RSVD_0 (Combined)
                    unsigned int SCDL : 1 = 1;             // Bit 7: Short Circuit in Discharge Latch
                };
            } pfAlertMaskB;
            static_assert(sizeof(pfAlertMaskB) == sizeof(uint8_t));

            /* BATRM.13.3.4.7/165 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int : 3;                      // Bits 0-2: RSVD_0 (Combined)
                    unsigned int LFOF : 1 = 0;             // Bit 3: Low Frequency Oscillator Failure
                    unsigned int VREF : 1 = 0;             // Bit 4: VREF Failure
                    unsigned int VSSF : 1 = 0;             // Bit 5: VSS Failure
                    unsigned int HWMX : 1 = 0;             // Bit 6: Hardware Max Alert
                    unsigned int : 1;                      // Bit 7: RSVD_0
                };
            } pfAlertMaskC;
            static_assert(sizeof(pfAlertMaskC) == sizeof(uint8_t));

            /* BATRM.13.3.4.8/165 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int TOSF : 1 = 0;             // Bit 0: Timeout Safety Fail
                    unsigned int : 7;                      // Bits 1-7: RSVD_0 (Combined)
                };
            } pfAlertMaskD;
            static_assert(sizeof(pfAlertMaskD) == sizeof(uint8_t));

        } Alarm;

        struct __attribute__((packed)) {

            /* BATRM.13.3.5.1/166 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int SUV : 1 = 0;              // Bit 0: Safety Cell Undervoltage Permanent Fail
                    unsigned int SOV : 1 = 0;              // Bit 1: Safety Cell Overvoltage Permanent Fail
                    unsigned int SOCC : 1 = 0;             // Bit 2: Safety Overcurrent in Charge Permanent Fail
                    unsigned int SOCD : 1 = 0;             // Bit 3: Safety Overcurrent in Discharge Permanent Fail
                    unsigned int SOT : 1 = 0;              // Bit 4: Safety Overtemperature Permanent Fail
                    unsigned int : 1;                      // Bit 5: RSVD_0
                    unsigned int SOTF : 1 = 0;             // Bit 6: Safety Overtemperature FET Permanent Fail
                    unsigned int CUDEP : 1 = 0;            // Bit 7: Copper Deposition Permanent Fail
                };
            } enabledPFA;
            static_assert(sizeof(enabledPFA) == sizeof(uint8_t));

            /* BATRM.13.3.5.2/ */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int CFETF : 1 = 0;            // Bit 0: Charge FET Permanent Fail
                    unsigned int DFETF : 1 = 0;            // Bit 1: Discharge FET Permanent Fail
                    unsigned int TWO_LVL : 1 = 0;          // Bit 2: Second Level Protector Permanent Fail (2LVL)
                    unsigned int VIMR : 1 = 0;             // Bit 3: Voltage Imbalance at Rest Permanent Fail
                    unsigned int VIMA : 1 = 0;             // Bit 4: Voltage Imbalance Active Permanent Fail
                    unsigned int : 2;                      // Bits 5-6: RSVD_0 (Combined)
                    unsigned int SCDL : 1 = 0;             // Bit 7: Short Circuit in Discharge Latch Permanent Fail
                };
            } enabledPFB;
            static_assert(sizeof(enabledPFB) == sizeof(uint8_t));

            /* BATRM.13.3.5.3/ */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int OTPF : 1 = 1;             // Bit 0: OTP Memory Permanent Fail
                    unsigned int DRMF : 1 = 1;             // Bit 1: Data ROM Permanent Fail
                    unsigned int IRMF : 1 = 1;             // Bit 2: Instruction ROM Permanent Fail
                    unsigned int LFOF : 1 = 0;             // Bit 3: Internal LFO Permanent Fail
                    unsigned int VREF : 1 = 0;             // Bit 4: Internal Voltage Reference Permanent Fail
                    unsigned int VSSF : 1 = 0;             // Bit 5: Internal VSS Measurement Permanent Fail
                    unsigned int HWMX : 1 = 0;             // Bit 6: Internal Stuck Hardware Mux Permanent Fail
                    unsigned int CMDF : 1 = 0;             // Bit 7: Commanded Permanent Fail
                };
            } enabledPFC;
            static_assert(sizeof(enabledPFC) == sizeof(uint8_t));

            /* BATRM.13.3.5.4/ */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int TOSF : 1 = 0;             // Bit 0: Top of Stack vs Cell Sum Permanent Fail
                    unsigned int : 7;                      // Bits 1-7: RSVD_0 (Combined)
                };
            } enabledPFD;
            static_assert(sizeof(enabledPFD) == sizeof(uint8_t));

        } PermanentFailure;

        struct __attribute__((packed)) {

            /* BATRM.13.3.6.1/ */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int SFET : 1 = 1;             // Bit 0: Series/Parallel FET mode (1=Series/Body diode protection enabled)
                    unsigned int SLEEPCHG : 1 = 0;         // Bit 1: CHG FET enabled in SLEEP mode (0=Disabled)
                    unsigned int HOST_FET_EN : 1 = 1;      // Bit 2: Host FET control commands allowed
                    unsigned int FET_CTRL_EN : 1 = 1;      // Bit 3: FETs are controlled by the device
                    unsigned int PDSG_EN : 1 = 0;          // Bit 4: Pre-discharge FET enabled
                    unsigned int FET_INIT_OFF : 1 = 0;     // Bit 5: Default host FET control state (0=FETs allowed to be on)
                    unsigned int : 2;                      // Bits 6-7: RSVD_0 (Combined)
                };
            } fetOptions;
            static_assert(sizeof(fetOptions) == sizeof(uint8_t));

            /* BATRM.13.3.6.2/ */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int CPEN : 1 = 1;             // Bit 0: Charge pumps for FET drivers are enabled
                    unsigned int LVEN : 1 = 0;             // Bit 1: Charge pump overdrive level (0=High/11V, 1=Low/5.5V)
                    unsigned int SFMODE_SLEEP : 1 = 0;     // Bit 2: DSG FET driver source-follower mode in SLEEP
                    unsigned int : 5;                      // Bits 3-7: RSVD_0 (Combined)
                };
            } chgPumpControl;
            static_assert(sizeof(chgPumpControl) == sizeof(uint8_t));

            /* BATRM.13.3.6.3/ */
            int16_t prechargeStartVoltage; // Default: 0 mV (0x0000)

            /* BATRM.13.3.6.4/ */
            int16_t prechargeStopVoltage;  // Default: 0 mV (0x0000)

            /* BATRM.13.3.6.6/ */
            uint8_t predischargeStopDelta; // Default: 50 (500 mV, in 10mV units)

        } FET;

        // dont care
//        struct __attribute__((packed)) {
//        } CurrentThreshold;

        struct __attribute__((packed)) {
            /* BATRM.13.3.8.1/170 */
            uint8_t checkTime_S;
        } CellOpenWire;

        struct __attribute__((packed)) {
            /* BATRM.13.3.9.1 - 13.3.9.16/ */
            // Each element is an I2 (signed 16-bit integer) representing resistance in mΩ.
            // Default value for all is 0 mΩ (0x0000).
            int16_t cellInterconnectResistance_mOhm[16]; // Indices 0 to 15 map to Cell 1 to Cell 16
        } InterconnectResistance;

        struct __attribute__((packed)) {
            /* BATRM.13.3.11.1/174 */
            union {
                uint8_t Raw;
                struct __attribute__((packed)) {
                    unsigned int CB_CHG : 1 = 0;           // Bit 0: Cell balancing allowed while charging
                    unsigned int CB_RLX : 1 = 0;           // Bit 1: Cell balancing allowed in relax conditions
                    unsigned int CB_SLEEP : 1 = 0;         // Bit 2: Cell balancing allowed in SLEEP mode
                    unsigned int CB_NOSLEEP : 1 = 0;       // Bit 3: SLEEP prevented while cell balancing is active
                    unsigned int CB_NO_CMD : 1 = 0;        // Bit 4: Host-controlled balancing commands are ignored
                    unsigned int : 3;                      // Bits 5-7: RSVD_0 (Combined)
                };
            } balancingConfiguration;
            static_assert(sizeof(balancingConfiguration) == sizeof(uint8_t));

            /* BATRM.13.3.11.2/ */
            int8_t minCellTemp_C; // Default: -20 °C (-0x14)

            /* BATRM.13.3.11.3/ */
            int8_t maxCellTemp_C; // Default: 60 °C (0x3C)

            /* BATRM.13.3.11.4/ */
            int8_t maxInternalTemp_C; // Default: 70 °C (0x46)

            /* BATRM.13.3.11.5/ */
            uint8_t cellBalanceInterval_s; // Default: 20 s (0x14)

            /* BATRM.13.3.11.6/ */
            uint8_t cellBalanceMaxCells; // Default: 1 Num (0x01)

            /* BATRM.13.3.11.7/ */
            int16_t cellBalanceMinCellV_Charge_mV; // Default: 3900 mV (0x0F3C)

            /* BATRM.13.3.11.8/ */
            uint8_t cellBalanceMinDelta_Charge_mV; // Default: 40 mV (0x28)

            /* BATRM.13.3.11.9/175 */
            uint8_t cellBalanceStopDelta_Charge_mV; // Default: 20 mV (0x14)

            /* BATRM.13.3.11.10/ */
            int16_t cellBalanceMinCellV_Relax_mV; // Default: 3900 mV (0x0F3C)

            /* BATRM.13.3.11.11/ */
            uint8_t cellBalanceMinDelta_Relax_mV; // Default: 40 mV (0x28)

            /* BATRM.13.3.11.12/ */
            uint8_t cellBalanceStopDelta_Relax_mV; // Default: 20 mV (0x14)

        } CellBalancingConfig;

        bool operator==(const BQ76952SSetting& other) const;
    };

    struct __attribute__((packed)) {
        struct __attribute__((packed)) {
            /* BATRM.13.4.1.1/ */
            int16_t shutdownCellVoltage_mV; // Default: 0 mV (0x0000)

            /* BATRM.13.4.1.2/ */
            int16_t shutdownStackVoltage_10mV; // Default: 600 (6000 mV or 6.0 V, 0x0258)

            /* BATRM.13.4.1.3/ */
            uint8_t lowVShutdownDelay_s; // Default: 1 s (0x01)

            /* BATRM.13.4.1.4/ */
            uint8_t shutdownTemperature_C; // Default: 85 °C (0x55)

            /* BATRM.13.4.1.5/ */
            uint8_t shutdownTemperatureDelay_s; // Default: 5 s (0x05)

            /* BATRM.13.4.1.6/ */
            uint8_t fetOffDelay_0_25s; // Default: 0 (0 seconds, 0x00)

            /* BATRM.13.4.1.8/ */
            uint8_t autoShutdownTime_min; // Default: 0 min (Disabled, 0x00)

            /* BATRM.13.4.1.9/ */
            uint8_t ramFailShutdownTime_s; // Default: 5 s (0x05)
        } Shutdown;

        struct __attribute__((packed)) {
            /* BATRM.13.4.2.1/ */
            int16_t sleepCurrent_mA; // Default: 20 mA (0x0014)

            /* BATRM.13.4.2.2/ */
            uint8_t voltageTime_s; // Default: 5 s (0x05)

            /* BATRM.13.4.2.3/ */
            int16_t wakeComparatorCurrent_mA; // Default: 500 mA (0x01F4)

            /* BATRM.13.4.2.4/ */
            uint8_t sleepHysteresisTime_s; // Default: 10 s (0x0A)

            /* BATRM.13.4.2.5/ */
            int16_t sleepChargerVoltageThreshold_10mV; // Default: 2000 (20.0 V, 0x07D0)

            /* BATRM.13.4.2.6/ */
            int16_t sleepChargerPackTosDelta_10mV; // Default: 200 (2.0 V, 0x00C8)
        } Sleep;

    } Power;

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
