/*
 * VT.cpp
 *
 *  Created on: Dec 9, 2025
 *      Author: turtl
 */

#include "VT.hpp"
#include "Core/system.hpp"


// variables
namespace VT {
    uint8_t id = 0;

    OpProfile_t opProfile = {
             .balancing_enable  = false,
             .cell_mV_min       = 1300,
             .cell_mV_max       = 4300,
        };

    OpVars_t opVars = {
        .bbqs = {
            {
                .bq = {
                    .spi = System::spi1,
                    .cs  = System::GPIO::PA14,
                },
                .resetPin = System::GPIO::PA15,
            }
        },
    };
}

namespace VT::BBQ {
    const BQ76952::BQ76952SSetting DEFAULT_BBQ_SETTING= {
             .Fuse = {
                 .minBlowFuseVoltage_10mV= 5000 / 10,   // 75V
                 .timeout_S              = 30,           // 0:indefinite
             },
             .Configuration = {
                 .powerConfig = {
                     .wake_speed     = 0,
                     .loop_slow      = 3,
                     .cb_loop_slow   = 2,
                     .fastadc        = 0,
                     .otsd           = 1,
                     .sleep          = 1,
                     .dpslp_lfo      = 0,
                     .dpslp_ldo      = 1,
                     .dpslp_pd       = 1,
                     .shut_ts2       = 0,
                     .dpslp_ot       = 1,
                 },
                 .REG12Config = {
                     .enable1    = 1,
                     .reg1_v     = 6, // use 2.5V logic (the line spikes sometimes)
                     .enable2    = 0,
                     .reg2_v     = 0,
                 },
                 .REG0Config = {
                     .enable0    = 1,
                 },
                 .HWDRegulatorOptions = { // TODO: is a safety thing, set up properly on final product
                     .toggle_time    = 5, // 5S power cycle
                     .toggle_opt     = 1,  // turn-off then turn-on on HWD
                 },
                 .spiConfig = {
                     .filt       = 1, // use digital filter?
                     .miso_reg1  = 1, // logic high V = REG1 output
                 },
                 .commIdleTime_S = 1, // 1S of no comms before turning off HFO
                 .cfetoffPinConfig = {// WARNING: effects SPI CS behavior
                     .Raw = 0,   // as SPI cs
                 },
                 .dfetoffPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                     .function  = 3,    // 3 : ADC/thermistor
                     .opt1_0    = 0b10, // thermistor reported, not used for anything else
                     .opt3_2    = 0b11, // use raw adc count
                     .opt5_4    = 0b00, // internal PU: 18k
                 },
                 .alertPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                     .function  = 3,    // 3 : ADC/thermistor
                     .opt1_0    = 0b10, // thermistor reported, not used for anything else
                     .opt3_2    = 0b11, // use raw adc count
                     .opt5_4    = 0b00, // internal PU: 18k
                 },
                 .TS1Config = { // TODO: figure out how to do custom thermistor polynomial
                     .function   = 3,    // as ADC/thermistor
                     .opt1_0     = 0b10, // thermistor reported, not used for anything else
                     .opt3_2     = 0b11, // use raw adc count
                     .opt5_4     = 0b00, // internal PU: 18k
                 },
                 .TS2Config = { // TODO: figure out how to do custom thermistor polynomial
                    .function   = 3,    // as ADC/thermistor
                    .opt1_0     = 0b10, // thermistor reported, not used for anything else
                    .opt3_2     = 0b11, // use raw adc count
                    .opt5_4     = 0b00, // internal PU: 18k
                 },
                 .TS3Config = { // TODO: figure out how to do custom thermistor polynomial
                    .function   = 3,    // as ADC/thermistor
                    .opt1_0     = 0b10, // thermistor reported, not used for anything else
                    .opt3_2     = 0b11, // use raw adc count
                    .opt5_4     = 0b00, // internal PU: 18k
                 },
                 .HDQPinConfig = { // WARNING: effects SPI MOSI behavior
                     .Raw = 0,   // as SPI MOSI
                 },
                 .DCHGPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                    .function   = 3,    // as ADC/thermistor
                    .opt1_0     = 0b10, // thermistor reported, not used for anything else
                    .opt3_2     = 0b11, // use raw adc count
                    .opt5_4     = 0b00, // internal PU: 18k
                 },
                 .DDSGPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                    .function   = 3,    // as ADC/thermistor
                    .opt1_0     = 0b10, // thermistor reported, not used for anything else
                    .opt3_2     = 0b11, // use raw adc count
                    .opt5_4     = 0b00, // internal PU: 18k
                 },
                 .DAConfig = {
                     .user_amps  = 2, // USER AMPS unit selection. 2:10mA,3:100mA . see userAto10mA macro
                     .user_volts = 1, // USER VOLTS unit selection. 0:1mV,1:10mV . see userVto10mV macro
                     .tint_en    = 1, // die temp used as a cell temp? 0:no, 1:yes.
                     .tint_fett  = 1, // die tmep used as fet temp? 0:no, 1:yes
                 },
                 .VcellMode  = 0b0011'1111'1111'1111,
                 .CC3Samples = 0x80,
             },

             .Protection = {
                 .protectionConfiguraiton = {
                    .PF_FETS         = 0, // Bit 1: PF causes FETs off (Default: 1)
                    .PF_REGS         = 0, // Bit 2: PF causes regulators off (Default: 0)
                    .PF_DPSLP        = 0, // Bit 3: PF causes device to enter DEEPSLEEP (Default: 0)
                    .PF_FUSE         = 0, // Bit 4: PF causes fuse blow (Default: 0)
                    .PF_OTP          = 0, // Bit 5: PF status preserved across reset/written to OTP (Default: 0)
                    .PACK_FUSE       = 0, // Bit 7: Use PACK voltage for Min Blow Fuse Voltage check (Default: 0)
                    .FETF_FUSE       = 0, // Bit 8: FETF bypasses Min Blow Fuse Voltage check (Default: 0)
                    .OCDL_CURR_RECOV = 0, // Bit 9: OCD Latch recovers based on charge current (Default: 0)
                    .SCDL_CURR_RECOV = 0, // Bit 10: SCD Latch recovers based on charge current (Default: 0)
                 },
                 .enabledProtectionsA = {
                     .CUV  = 1, // Bit 2: Cell Undervoltage Protection (Default: 0)
                     .COV  = 1, // Bit 3: Cell Overvoltage Protection (Default: 1)
                     .OCC  = 0, // Bit 4: Overcurrent in Charge Protection (Default: 0)
                     .OCD1 = 0, // Bit 5: Overcurrent in Discharge 1st Tier Protection (Default: 0)
                     .OCD2 = 0, // Bit 6: Overcurrent in Discharge 2nd Tier Protection (Default: 0)
                     .SCD  = 0, // Bit 7: Short Circuit in Discharge Protection (Default: 1)
                 },
                 .enabledProtectionsB = {
                     .UTC   = 0, // Bit 0: Undertemperature in Charge (Default: 0)
                     .UTD   = 0, // Bit 1: Undertemperature in Discharge (Default: 0)
                     .UTINT = 0, // Bit 2: Internal Undertemperature (Default: 0)
                     .OTC   = 0, // Bit 4: Overtemperature in Charge (Default: 0)
                     .OTD   = 0, // Bit 5: Overtemperature in Discharge (Default: 0)
                     .OTINT = 1, // Bit 6: Internal Overtemperature (Default: 0)
                     .OTF   = 0, // Bit 7: FET Overtemperature (Default: 0)
                 },
                 .enabledProtectionsC = {
                     .HWDF = 0, // Bit 1: Host Watchdog Fault (Default: 0)
                     .PTO  = 0, // Bit 2: Precharge Timeout (Default: 0)
                     .COVL = 0, // Bit 4: Cell Overvoltage Latch (Default: 0)
                     .OCDL = 0, // Bit 5: Overcurrent in Discharge Latch (Default: 0)
                     .SCDL = 0, // Bit 6: Short Circuit in Discharge Latch (Default: 0)
                     .OCD3 = 0, // Bit 7: Overcurrent in Discharge 3rd Tier Protection (Default: 0)
                 },
                 .chgFetProtectionsA = {
                     .COV  = 1, // Bit 3: Cell Overvoltage Protection (Default: 1)
                     .OCC  = 1, // Bit 4: Overcurrent in Charge Protection (Default: 1)
                     .SCD  = 1, // Bit 7: Short Circuit in Discharge Protection (Default: 1)
                 },
                 .chgFetProtectionsB = {
                     // Note: Reserved/Anonymous fields (Bits 1, 3, 5) are implicitly zeroed by C++20 standard
                     .UTC   = 1, // Bit 0: Undertemperature in Charge (Default: 1)
                     .UTINT = 1, // Bit 2: Internal Undertemperature (Default: 1)
                     .OTC   = 1, // Bit 4: Overtemperature in Charge (Default: 1)
                     .OTINT = 1, // Bit 6: Internal Overtemperature (Default: 1)
                     .OTF   = 1, // Bit 7: FET Overtemperature (Default: 1)
                 },
                 .chgFetProtectionsC = {
                     .HWDF = 1, // Bit 1: Host Watchdog Fault (Default: 1)
                     .PTO  = 1, // Bit 2: Precharge Timeout (Default: 1)
                     .COVL = 1, // Bit 4: Cell Overvoltage Latch (Default: 1)
                     .SCDL = 1, // Bit 6: Short Circuit in Discharge Latch (Default: 1)
                 },
                 .dsgFetProtectionsA = {
                     .CUV  = 1, // Bit 2: Cell Undervoltage Protection (Default: 1)
                     .OCD1 = 1, // Bit 5: Overcurrent in Discharge 1st Tier Protection (Default: 1)
                     .OCD2 = 1, // Bit 6: Overcurrent in Discharge 2nd Tier Protection (Default: 1)
                     .SCD  = 1, // Bit 7: Short Circuit in Discharge Protection (Default: 1)
                 },
                 .dsgFetProtectionsB = {
                     .UTD   = 1, // Bit 1: Undertemperature in Discharge (Default: 1)
                     .UTINT = 1, // Bit 2: Internal Undertemperature (Default: 1)
                     .OTD   = 1, // Bit 5: Overtemperature in Discharge (Default: 1)
                     .OTINT = 1, // Bit 6: Internal Overtemperature (Default: 1)
                     .OTF   = 1, // Bit 7: FET Overtemperature (Default: 1)
                 },
                 .dsgFetProtectionsC = {
                     .HWDF = 1, // Bit 1: Host Watchdog Fault (Default: 1)
                     .OCDL = 1, // Bit 5: Overcurrent in Discharge Latch (Default: 1)
                     .SCDL = 1, // Bit 6: Short Circuit in Discharge Latch (Default: 1)
                     .OCD3 = 1, // Bit 7: Overcurrent in Discharge 3rd Tier Protection (Default: 1)
                 },
             },

             .Alarm = {
                 .defaultAlarmMask = {
                     .WAKE          = 0, // Bit 0: Device wakened from SLEEP mode (Default: 0)
                     .ADSCAN        = 0, // Bit 1: Voltage ADC Scan Complete (Default: 0)
                     .CB            = 0, // Bit 2: Cell balancing is active (Default: 0)
                     .FUSE          = 0, // Bit 3: FUSE Pin Driven (Default: 0)
                     .SHUTV         = 0, // Bit 4: Stack voltage below Shutdown Stack Voltage (Default: 0)
                     .XDSG          = 0, // Bit 5: DSG FET is off (Default: 0)
                     .XCHG          = 0, // Bit 6: CHG FET is off (Default: 0)
                     .FULLSCAN      = 0, // Bit 7: Full Voltage Scan Complete (Default: 0)
                     .INITCOMP      = 0, // Bit 9: Initialization completed (Default: 0)
                     .INITSTART     = 0, // Bit 10: Initialization started (Default: 0)
                     .MSK_PFALERT   = 1, // Bit 11: Permanent Fail alert triggered and masked (Default: 1)
                     .MSK_SFALERT   = 1, // Bit 12: Safety alert triggered and masked (Default: 1)
                     .PF            = 1, // Bit 13: Enabled Permanent Fail fault triggers (Default: 1)
                     .SSA           = 1, // Bit 14: Bit is set in Safety Status A (Default: 1)
                     .SSBC          = 1, // Bit 15: Bit is set in Safety Status B/C (Default: 1)
                 },
                 .sfAlertMaskA = {
                     .CUV  = 1, // Bit 2: Cell Undervoltage Protection (Default: 1)
                     .COV  = 1, // Bit 3: Cell Overvoltage Protection (Default: 1)
                     .OCC  = 0, // Bit 4: Overcurrent in Charge Protection (Default: 1)
                     .OCD1 = 0, // Bit 5: Overcurrent in Discharge 1st Tier Protection (Default: 1)
                     .OCD2 = 0, // Bit 6: Overcurrent in Discharge 2nd Tier Protection (Default: 1)
                     .SCD  = 0, // Bit 7: Short Circuit in Discharge Protection (Default: 1)
                 },
                 .sfAlertMaskB = {
                     .UTC   = 0, // Bit 0: Undertemperature in Charge (Default: 1)
                     .UTD   = 0, // Bit 1: Undertemperature in Discharge (Default: 1)
                     .UTINT = 0, // Bit 2: Internal Undertemperature (Default: 1)
                     .OTC   = 1, // Bit 4: Overtemperature in Charge (Default: 1)
                     .OTD   = 1, // Bit 5: Overtemperature in Discharge (Default: 1)
                     .OTINT = 1, // Bit 6: Internal Overtemperature (Default: 1)
                     .OTF   = 0, // Bit 7: FET Overtemperature (Default: 1)
                 },
                 .sfAlertMaskC = {
                     .PTO  = 0, // Bit 2: Precharge Timeout (Default: 1)
                     .COVL = 0, // Bit 4: Cell Overvoltage Latch (Default: 1)
                     .OCDL = 0, // Bit 5: Overcurrent in Discharge Latch (Default: 1)
                     .SCDL = 0, // Bit 6: Short Circuit in Discharge Latch (Default: 1)
                     .OCD3 = 0, // Bit 7: Overcurrent in Discharge 3rd Tier Protection (Default: 1)
                 },
                 .pfAlertMaskA = {
                     .SUV   = 1, // Bit 0: Safety Undervoltage (Default: 1)
                     .SOV   = 1, // Bit 1: Safety Overvoltage (Default: 1)
                     .SOCC  = 0, // Bit 2: Safety Overcurrent in Charge (Default: 1)
                     .SOCD  = 0, // Bit 3: Safety Overcurrent in Discharge (Default: 1)
                     .SOT   = 1, // Bit 4: Safety Overtemperature (Default: 1)
                     .SOTF  = 0, // Bit 6: Safety Over-FET-Temperature (Default: 1)
                     .CUDEP = 1, // Bit 7: Cell Undervoltage Depletion (Default: 0)
                 },
                 .pfAlertMaskB = {
                     .CFETF   = 0, // Bit 0: CHG FET Fault (Default: 1)
                     .DFETF   = 0, // Bit 1: DSG FET Fault (Default: 1)
                     .TWO_LVL = 0, // Bit 2: 2nd Level Protection (2LVL) (Default: 1)
                     .VIMR    = 1, // Bit 3: Voltage Imbalance Measurement Range (Default: 1)
                     .VIMA    = 1, // Bit 4: Voltage Imbalance Measurement Area (Default: 1)
                     .SCDL    = 1, // Bit 7: Short Circuit in Discharge Latch (Default: 1)
                 },
                 .pfAlertMaskC = {
                     .LFOF = 0, // Bit 3: Low Frequency Oscillator Failure (Default: 0)
                     .VREF = 0, // Bit 4: VREF Failure (Default: 0)
                     .VSSF = 0, // Bit 5: VSS Failure (Default: 0)
                     .HWMX = 0, // Bit 6: Hardware Max Alert (Default: 0)
                 },
                 .pfAlertMaskD = {
                     .TOSF = 0, // Bit 0: Timeout Safety Fail (Default: 0)
                 },
             },

             .PermanentFailure = {
                  .enabledPFA = {
                      .SUV   = 0, // Bit 0: Safety Cell Undervoltage Permanent Fail (Default: 0)
                      .SOV   = 0, // Bit 1: Safety Cell Overvoltage Permanent Fail (Default: 0)
                      .SOCC  = 0, // Bit 2: Safety Overcurrent in Charge Permanent Fail (Default: 0)
                      .SOCD  = 0, // Bit 3: Safety Overcurrent in Discharge Permanent Fail (Default: 0)
                      .SOT   = 0, // Bit 4: Safety Overtemperature Permanent Fail (Default: 0)
                      .SOTF  = 0, // Bit 6: Safety Overtemperature FET Permanent Fail (Default: 0)
                      .CUDEP = 0, // Bit 7: Copper Deposition Permanent Fail (Default: 0)
                  },
                  .enabledPFB = {
                      .CFETF   = 0, // Bit 0: Charge FET Permanent Fail (Default: 0)
                      .DFETF   = 0, // Bit 1: Discharge FET Permanent Fail (Default: 0)
                      .TWO_LVL = 0, // Bit 2: Second Level Protector Permanent Fail (2LVL) (Default: 0)
                      .VIMR    = 0, // Bit 3: Voltage Imbalance at Rest Permanent Fail (Default: 0)
                      .VIMA    = 0, // Bit 4: Voltage Imbalance Active Permanent Fail (Default: 0)
                      .SCDL    = 0, // Bit 7: Short Circuit in Discharge Latch Permanent Fail (Default: 0)
                  },
                  .enabledPFC = {
                      .OTPF = 1, // Bit 0: OTP Memory Permanent Fail (Default: 1)
                      .DRMF = 1, // Bit 1: Data ROM Permanent Fail (Default: 1)
                      .IRMF = 1, // Bit 2: Instruction ROM Permanent Fail (Default: 1)
                      .LFOF = 0, // Bit 3: Internal LFO Permanent Fail (Default: 0)
                      .VREF = 0, // Bit 4: Internal Voltage Reference Permanent Fail (Default: 0)
                      .VSSF = 0, // Bit 5: Internal VSS Measurement Permanent Fail (Default: 0)
                      .HWMX = 0, // Bit 6: Internal Stuck Hardware Mux Permanent Fail (Default: 0)
                      .CMDF = 0, // Bit 7: Commanded Permanent Fail (Default: 0)
                  },
             },

             // we dotn use this, values are just what ever it takes to turn stuff off
             .FET = {
                 .fetOptions = {
                     .SFET          = 0, // Bit 0: Series/Parallel FET mode (Default: 1)
                     .SLEEPCHG      = 0, // Bit 1: CHG FET enabled in SLEEP mode (Default: 0)
                     .HOST_FET_EN   = 0, // Bit 2: Host FET control commands allowed (Default: 1)
                     .FET_CTRL_EN   = 0, // Bit 3: FETs are controlled by the device (Default: 1)
                     .PDSG_EN       = 0, // Bit 4: Pre-discharge FET enabled (Default: 0)
                     .FET_INIT_OFF  = 0, // Bit 5: Default host FET control state (Default: 0)
                 },
                 .chgPumpControl = { // WARNING: do not turn on charge pump when stack over 35V
                     .CPEN           = 0, // Bit 0: Charge pumps for FET drivers are enabled (Default: 1)
                     .LVEN           = 1, // Bit 1: Charge pump overdrive level (Default: 0 / High/11V)
                     .SFMODE_SLEEP   = 0, // Bit 2: DSG FET driver source-follower mode in SLEEP (Default: 0)
                 },
                 .prechargeStartVoltage  = 0,
                 .prechargeStopVoltage   = 0,
                 .predischargeStopDelta  = 50, //  default
             },

             .CellOpenWire = {
                  .checkTime_S = 10
             },

             .InterconnectResistance = {
                  .cellInterconnectResistance_mOhm = {
                      0,0,0,0,
                      0,0,0,0,
                      0,0,0,0,
                      0,0,0,0,
     //                 3500,3500,3500,3500,
     //                 3500,3500,3500,3500,
     //                 3500,3500,3500,3500,
     //                 3500,3500,3500,3500,
                  },
             },

             .CellBalancingConfig = {
                  .balancingConfiguration = {
                      .CB_CHG     = 1, // Bit 0: Cell balancing allowed while charging (Default: 0)
                      .CB_RLX     = 1, // Bit 1: Cell balancing allowed in relax conditions (Default: 0)
                      .CB_SLEEP   = 1, // Bit 2: Cell balancing allowed in SLEEP mode (Default: 0)
                      .CB_NOSLEEP = 0, // Bit 3: SLEEP prevented while cell balancing is active (Default: 0)
                      .CB_NO_CMD  = 0, // Bit 4: Host-controlled balancing commands are ignored (Default: 0)
                  },
                  .minCellTemp_C     = -20,
                  .maxCellTemp_C     = 60,
                  .maxInternalTemp_C = 80,
                  .cellBalanceInterval_s = 5,
                  .cellBalanceMaxCells   = 16,
                  .cellBalanceMinCellV_Charge_mV = 2000,
                  .cellBalanceMinDelta_Charge_mV = 50,
                  .cellBalanceStopDelta_Charge_mV= 10,
                  .cellBalanceMinCellV_Relax_mV  = 2200,
                  .cellBalanceMinDelta_Relax_mV  = 20,
                  .cellBalanceStopDelta_Relax_mV = 10,
             },
         };

}

VT::OpVars_t::BBQ_t & VT::getSelectedBBQ() {
    if(opVars.user_selected_BQ > NUM_BBQs)
        opVars.user_selected_BQ = NUM_BBQs - 1;
    return opVars.bbqs[opVars.user_selected_BQ];
}

void VT::preScheduler_init(){
    // dont put stuff here unless it needs to be here
}

void VT::postScheduler_init(){
    {
       using namespace VT;
       opVars.user_selected_BQ = 0;
       opVars.HRLV_IL_sw_dsrd = false;

       for(auto & i : opVars.bbqs) {
           i.stack_10mV = 0;
           i.die_10mCl = 0;
           i.cell_balancing_status = 0;
           for(auto & j : i.cell_10mCl)
               j = 0;
           for(auto & j : i.cell_mV)
               j = 0;
       }
   }
}

/** write to non volatile storage */
bool VT::BBQ::storeSetting(buffersize_t i, BQ76952::BQ76952SSetting const *) {
    return false;
}

/** read from non volatile storage */
bool VT::BBQ::recalSetting(buffersize_t i, BQ76952::BQ76952SSetting *) {
    return false;
}

/** write to BBQ */
bool VT::BBQ::applySetting(BQ76952 & bq, BQ76952::BQ76952SSetting const *) {
    return false;
}

/** read from BBQ */
bool VT::BBQ::retreiveSetting(BQ76952 const & bq, BQ76952::BQ76952SSetting *) {
    return false;
}

uint8_t VT::getID() {
    return System::mcuID;
}
