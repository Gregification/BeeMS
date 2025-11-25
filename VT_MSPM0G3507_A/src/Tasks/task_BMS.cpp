/*
 * task_BMS.cpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 *
 *  All voltage tap logic is in here
 */

#include <stdio.h>

#include <Tasks/task_BMS.hpp>
#include "Core/system.hpp"
#include "Core/FancyCli.hpp"
#include "Middleware/BQ769x2/BQ76952.hpp"
#include "Core/common.h"
#include "Core/BMS/BMSComms.hpp"   // adjust path if needed


// user amp to centiamp scale (10mA)
// using 10mA scale (determined by DAConfiguration)
#define userAto10mA(X) (X)

// user volt to centivolt scale (10mV)
// using 10mV scale (determined by DAConfiguration)
#define userVto10mV(X) (X)

// only 1 BQ on the voltage tap board
BQ76952 bq = {
        .spi  = &System::spi1,
        .cs   = &System::GPIO::PB15,
    };
auto &bqReset = System::GPIO::PA15;

BQ76952::BQ76952SSetting constexpr bqSetting = {
        .Fuse = {
            .minBlowFuseVoltage_10mV= 5000 / 10,   // 75V
            .timeout_S              = 30,           // 0:indefinite
        },
        .Configuration = {
            .powerConfig = {
                .wake_speed     = 0,
                .loop_slow      = 0,
                .cb_loop_slow   = 2,
                .fastadc        = 0,
                .otsd           = 1,
                .sleep          = 1,
                .dpslp_lfo      = 0,
                .dpslp_ldo      = 0,
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
                .toggle_time    = 0, // 5S power cycle
                .toggle_opt     = 0,  // turn-off then turn-on on HWD
            },
            .spiConfig = {
                .filt       = 1, // use digital filter?
                .miso_reg1  = 1, // logic high V = REG1 output
            },
            .commIdleTime_S = 0, // 1S of no comms before turning off HFO
            .cfetoffPinConfig = {// WARNING: effects SPI CS behavior
                .Raw = 0,   // as SPI cs
            },
            .dfetoffPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                .Raw = 0,
            },
            .alertPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                .Raw = 0,
            },
            .TS1Config = { // TODO: figure out how to do custom thermistor polynomial
                .function   = 3,    // as ADC/thermistor
                .opt1_0     = 0b01, // as thermistor
                .opt3_2     = 0b00, // 18k model
                .opt5_4     = 0b00, // 18k PU
            },
            .TS2Config = { // TODO: figure out how to do custom thermistor polynomial
                .Raw = 0,
            },
            .TS3Config = { // TODO: figure out how to do custom thermistor polynomial
                .Raw = 0,
            },
            .HDQPinConfig = { // WARNING: effects SPI MOSI behavior
                .Raw = 0,   // as SPI MOSI
            },
            .DCHGPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                .Raw = 0,
            },
            .DDSGPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                .Raw = 0,
            },
            .DAConfig = {
                .user_amps  = 2, // USER AMPS unit selection. 2:10mA,3:100mA . see userAto10mA macro
                .user_volts = 1, // USER VOLTS unit selection. 0:1mV,1:10mV . see userVto10mV macro
                .tint_en    = 1, // die temp used as a cell temp? 0:no, 1:yes.
                .tint_fett  = 1, // die tmep used as fet temp? 0:no, 1:yes
            },
            .VcellMode  = 0b0000'1111'1111'1111,
            .CC3Samples = 0x80,
        },

        .Protection = {
            .protectionConfiguraiton = {
               .PF_FETS         = 1, // Bit 1: PF causes FETs off (Default: 1)
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
                .CUV  = 0, // Bit 2: Cell Undervoltage Protection (Default: 0)
                .COV  = 1, // Bit 3: Cell Overvoltage Protection (Default: 1)
                .OCC  = 0, // Bit 4: Overcurrent in Charge Protection (Default: 0)
                .OCD1 = 0, // Bit 5: Overcurrent in Discharge 1st Tier Protection (Default: 0)
                .OCD2 = 0, // Bit 6: Overcurrent in Discharge 2nd Tier Protection (Default: 0)
                .SCD  = 1, // Bit 7: Short Circuit in Discharge Protection (Default: 1)
            },
            .enabledProtectionsB = {
                .UTC   = 0, // Bit 0: Undertemperature in Charge (Default: 0)
                .UTD   = 0, // Bit 1: Undertemperature in Discharge (Default: 0)
                .UTINT = 0, // Bit 2: Internal Undertemperature (Default: 0)
                .OTC   = 0, // Bit 4: Overtemperature in Charge (Default: 0)
                .OTD   = 0, // Bit 5: Overtemperature in Discharge (Default: 0)
                .OTINT = 0, // Bit 6: Internal Overtemperature (Default: 0)
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
             .checkTime_S = 5
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
             .maxInternalTemp_C = 70,
             .cellBalanceInterval_s = 20,
             .cellBalanceMaxCells   = 10,
             .cellBalanceMinCellV_Charge_mV = 3000,
             .cellBalanceMinDelta_Charge_mV = 30,
             .cellBalanceStopDelta_Charge_mV= 10,
             .cellBalanceMinCellV_Relax_mV  = 3000,
             .cellBalanceMinDelta_Relax_mV  = 30,
             .cellBalanceStopDelta_Relax_mV = 10,
        },
    };

//bool setup_BBQ(BQ76952 & b);

void Task::BMS_task(void *){
    System::uart_ui.nputs(ARRANDN("BMS_task start" NEWLINE));

    //---- SPI setup ------------------------------------------
    bq.spi->setSCLKTarget(250e3); // bq76952 max speed of 2MHz
    DL_GPIO_enableOutput(GPIOPINPUX(*bq.cs)); // SPI CS
    DL_GPIO_initDigitalOutputFeatures(
            bq.cs->iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_enableOutput(GPIOPINPUX(bqReset)); // BBQ reset
    DL_GPIO_initDigitalOutputFeatures(
            bqReset.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );

    bq.cs->clear();
    bqReset.set();

    vTaskDelay(pdMS_TO_TICKS(500)); // CS needs some time to get recognized by the slave

    bqReset.clear();

    vTaskDelay(pdMS_TO_TICKS(300));

    //--- init BQ76952 ----------------------------------------
    // TODO: on the final product we need to somehow prevent the MCU from locking up the BQ,
    //      if the MCU gets in a power cycle loop.

//    bq.unseal(0x36720414);

    if(! bq.setConfig(&bqSetting))
        System::FailHard("failed to init BBQ settings on MCU power up. failed to write");

    while(1){

        BQ76952::DAStatus5 dastatus5;
        {
            bool success = bq.sendSubcommandR(BQ769X2_PROTOCOL::Cmd::DASTATUS5, &dastatus5, sizeof(dastatus5));

            // Print Header
            System::uart_ui.nputs(ARRANDN(CLICLEAR CLIRESET "--- Battery Data Block ---" NEWLINE));

            // 1. VREG18 (16-bit)
            System::uart_ui.nputs(ARRANDN("VREG18 (ADC counts): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.Vreg18));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 2. VSS (16-bit)
            System::uart_ui.nputs(ARRANDN("VSS (ADC counts): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.VSS));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 3. Max Cell Voltage (16-bit)
            System::uart_ui.nputs(ARRANDN("Max Cell Voltage (mV): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.MaxCellVoltage));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 4. Min Cell Voltage (16-bit)
            System::uart_ui.nputs(ARRANDN("Min Cell Voltage (mV): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.MinCellVoltage));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 5. Battery Voltage Sum (16-bit)
            System::uart_ui.nputs(ARRANDN("Battery Voltage Sum (cV): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.BatteryVoltageSum));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 6. Avg Cell Temperature (16-bit)
            System::uart_ui.nputs(ARRANDN("Avg Cell Temperature (0.1 K): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.AvgCellTemperature));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 7. FET Temperature (16-bit)
            System::uart_ui.nputs(ARRANDN("FET Temperature (0.1 K): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.FETTemperature));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 8. Max Cell Temperature (16-bit)
            System::uart_ui.nputs(ARRANDN("Max Cell Temperature (0.1 K): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.MaxCellTemperature));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 9. Min Cell Temperature (16-bit)
            System::uart_ui.nputs(ARRANDN("Min Cell Temperature (0.1 K): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.MinCellTemperature));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 10. Second Avg Cell Temperature (16-bit)
            System::uart_ui.nputs(ARRANDN("Avg Cell Temp 2 (0.1 K): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.SecondAvgCellTemperature));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 11. CC3 Current (16-bit)
            System::uart_ui.nputs(ARRANDN("CC3 Current (userA): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.CC3Current));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 12. CC1 Current (16-bit)
            System::uart_ui.nputs(ARRANDN("CC1 Current (userA): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(dastatus5.CC1Current));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 13. CC2 Counts (32-bit)
            System::uart_ui.nputs(ARRANDN("CC2 Counts (ADC counts): "));
            System::uart_ui.putu32d(dastatus5.CC2Counts);
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 14. CC3 Counts (32-bit)
            System::uart_ui.nputs(ARRANDN("CC3 Counts (ADC counts): "));
            System::uart_ui.putu32d(dastatus5.CC3Counts);
            System::uart_ui.nputs(ARRANDN(NEWLINE));
        }

        BQ76952::DAStatus6 dastatus6;
        {

            // Read the data block for DASTATUS6 (Subcommand 0x0076)
            bool success = bq.sendSubcommandR(BQ769X2_PROTOCOL::Cmd::DASTATUS6, &dastatus6, sizeof(dastatus6));

            // Print Header
            System::uart_ui.nputs(ARRANDN(CLIRESET "--- Battery Data Block 6 ---" NEWLINE));

            // 1. Accum Charge (32-bit, signed)
            System::uart_ui.nputs(ARRANDN("Accum Charge (userAh): "));
            System::uart_ui.put32d(dastatus6.AccumCharge);
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 2. Accum Charge Fraction (32-bit, unsigned)
            System::uart_ui.nputs(ARRANDN("Accum Charge Fraction (U4): "));
            System::uart_ui.put32d(dastatus6.AccumChargeFraction);
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 3. Accum Time (32-bit, unsigned)
            System::uart_ui.nputs(ARRANDN("Accum Time (s): "));
            System::uart_ui.putu32d(dastatus6.AccumTime);
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 4. CFETOFF Counts (32-bit, signed)
            System::uart_ui.nputs(ARRANDN("CFETOFF Counts (ADC counts): "));
            System::uart_ui.putu32d(dastatus6.CFETOFF_Counts);
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 5. DFETOFF Counts (32-bit, signed)
            System::uart_ui.nputs(ARRANDN("DFETOFF Counts (ADC counts): "));
            System::uart_ui.putu32d(dastatus6.DFETOFF_Counts);
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 6. ALERT Counts (32-bit, signed)
            System::uart_ui.nputs(ARRANDN("ALERT Counts (ADC counts): "));
            System::uart_ui.putu32d(dastatus6.ALERT_Counts);
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 7. TS1 Counts (32-bit, signed)
            System::uart_ui.nputs(ARRANDN("TS1 Counts (ADC counts): "));
            System::uart_ui.putu32d(dastatus6.TS1_Counts);
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 8. TS2 Counts (32-bit, signed)
            System::uart_ui.nputs(ARRANDN("TS2 Counts (ADC counts): "));
            System::uart_ui.putu32d(dastatus6.TS2_Counts);
            System::uart_ui.nputs(ARRANDN(NEWLINE));
        }

        // print out all cell V's
        System::uart_ui.nputs(ARRANDN(CLIRESET NEWLINE "--- other stuff ---"));
        uint16_t cellmV[16];
        {

            System::uart_ui.nputs(ARRANDN(CLIRESET NEWLINE "CELLS: "));
            for(uint8_t i = 0; i < 16; i++){
                System::uart_ui.nputs(ARRANDN(CLIRESET "\t"));

                if(bqSetting.Configuration.VcellMode & BV(i))
                    System::uart_ui.nputs(ARRANDN(CLIYES));

                System::uart_ui.putu32d(i);
                System::uart_ui.nputs(ARRANDN(CLIRESET));
            }

            System::uart_ui.nputs(ARRANDN(CLIRESET NEWLINE "mV:    "));
            for(uint8_t i = 0; i < 16; i++){
                System::uart_ui.nputs(ARRANDN(CLIRESET "\t"));

                if(!bq.sendDirectCommandR(
                        (BQ769X2_PROTOCOL::CmdDrt)(BQ769X2_PROTOCOL::CmdDrt::Cell1Voltage + 2 * i),
                        cellmV + i,
                        sizeof(cellmV[0])
                    ))
                    System::uart_ui.nputs(ARRANDN(CLIBAD));

                System::uart_ui.putu32d(cellmV[i]);
            }

        }

        System::uart_ui.nputs(ARRANDN(CLIRESET NEWLINE  "bal?:  "));
        uint16_t activeBalMask = 0;
        {
            if(!bq.sendCommandR(
                    BQ769X2_PROTOCOL::Cmd::CB_ACTIVE_CELLS,
                    &activeBalMask,
                    sizeof(activeBalMask)
                )){
                System::uart_ui.nputs(ARRANDN(CLIBAD));
            } else {
                for(uint8_t i = 0; i < 16; i++){
                    System::uart_ui.nputs(ARRANDN("\t"));
                    System::uart_ui.putu32d((activeBalMask & BV(i)) != 0);
                }
            }
        }

        uint16_t internalTempRaw = 0;
        {
            System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "internal temp (0.1 K): "));
            if(!bq.sendDirectCommandR(
                BQ769X2_PROTOCOL::CmdDrt::IntTemperature,  // 0x68
                &internalTempRaw,
                sizeof(internalTempRaw)
            ))
                System::uart_ui.nputs(ARRANDN(CLIBAD));

            System::uart_ui.put32d(internalTempRaw);

        }

        uint16_t balTime = 0;
        {
            System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "bal time: "));
            if(!bq.sendSubcommandR(
                BQ769X2_PROTOCOL::Cmd::CBSTATUS1,
                &balTime,
                sizeof(balTime)
            ))
                System::uart_ui.nputs(ARRANDN(CLIBAD));

            System::uart_ui.putu32d(balTime);
        }


        // pack stuff and tx
        {
            DL_MCAN_TxBufElement txmsg = {
                .id     = 0x1,      // CAN id, 11b->[28:18], 29b->[] when using 11b can id
                .rtr    = 0,        // 0: data frame, 1: remote frame
                .xtd    = 1,        // 0: 11b id, 1: 29b id
                .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
                .dlc    = 3,        // data byte count, see DL comments
                .brs    = 0,        // 0: no bit rate switching, 1: yes brs
                .fdf    = 1,        // FD format, 0: classic CAN, 1: CAN FD format
                .efc    = 0,        // 0: dont store Tx events, 1: store
                .mm     = 0x3,      // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
            };

            BMSComms::PacketHeader * header = reinterpret_cast<BMSComms::PacketHeader *>(txmsg.data);
            header->typeSM = BMSComms::PktTypeSM_t::TEST_1;
            header->slaveID = 1;

            BMSComms::PktSM_Test1 * pktms = reinterpret_cast<BMSComms::PktSM_Test1 *>(header->data);
            { // populate
                for(uint8_t i = 0; i < 16; i++){
                    pktms->cellInfo[i].cellmV = cellmV[i];
                    pktms->cellInfo[i].balancing = (BV(i) & activeBalMask) != 0;
                }
                pktms->MaxCellVoltage = dastatus5.MaxCellVoltage;
                pktms->MinCellVoltage = dastatus5.MinCellVoltage;
                pktms->BatteryVoltageSum = dastatus5.BatteryVoltageSum;
            }

            txmsg.dlc = System::CANFD::len2DLC(sizeof(*pktms)) + sizeof(*header);

            DL_MCAN_TxFIFOStatus tf;
            DL_MCAN_getTxFIFOQueStatus(CANFD0, &tf);

            uint32_t bufferIndex = tf.putIdx;

            DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, bufferIndex, &txmsg);
            DL_MCAN_TXBufAddReq(CANFD0, tf.getIdx);

            System::uart_ui.nputs(ARRANDN("meow" NEWLINE));
        }



        vTaskDelay(pdMS_TO_TICKS(2e3));
    }

    /* pesudo code
     * {
     *      - POST
     *
     *      - use existing model if available
     *
     *      while(1){
     *
     *          - DAQ
     *
     *          - safety check
     *              - cell balance
     *              - fan pwm
     *
     *          - iterate model
     *              - rebase model if needed
     *              - periodically save model, like ~1h
     *
     *          - periodically/as-necessary TX to CAN ...
     *              - ~1s, DAQ
     *              - ~5s, safe operating ranges
     *
     *          - check and respond to ...
     *              - CAN
     *      }
     * }
     */


//    DL_MCAN_TxBufElement txmsg = {
//           .id     = 0x1,      // CAN id, 11b->[28:18], 29b->[] when using 11b can id
//           .rtr    = 0,        // 0: data frame, 1: remote frame
//           .xtd    = 1,        // 0: 11b id, 1: 29b id
//           .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
//           .dlc    = 8,        // data byte count, see DL comments
//           .brs    = 0,        // 0: no bit rate switching, 1: yes brs
//           .fdf    = 0,        // FD format, 0: classic CAN, 1: CAN FD format
//           .efc    = 0,        // 0: dont store Tx events, 1: store
//           .mm     = 0x3,      // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
//       };
//
////    while(1){
//        for(uint8_t i = 0; i < sizeof(cmds); i++){
//            uint16_t v = 0xBEEF;
//            bool success = bq.sendDirectCommandR(cmds[i], &v);
//            vTaskDelay(pdMS_TO_TICKS(1e6));
//
//            snprintf(ARRANDN(str), "%6d,", v);
//
//            System::uart_ui.nputs(ARRANDN(str));
//
//            if(!success)
//                v = 0;
//
//            ((uint16_t*)txmsg.data)[i % 8] = v;
//
//            // transmit in 8B packets
//            if((i+1) % 8 == 0)
//            {
//               DL_MCAN_TxFIFOStatus tf;
//               DL_MCAN_getTxFIFOQueStatus(CANFD0, &tf);
//
//               uint32_t bufferIndex = tf.putIdx;
//
//               DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, bufferIndex, &txmsg);
//               DL_MCAN_TXBufAddReq(CANFD0, tf.getIdx);
//            }
//        }
//        System::uart_ui.nputs(ARRANDN(NEWLINE));
//
//        vTaskDelay(pdMS_TO_TICKS(1e6));
//    }


    while(1)
    {
        // Canned CAN TX
        DL_MCAN_TxBufElement txmsg = {
                .id     = 0x1,      // CAN id, 11b->[28:18], 29b->[] when using 11b can id
                .rtr    = 0,        // 0: data frame, 1: remote frame
                .xtd    = 1,        // 0: 11b id, 1: 29b id
                .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
                .dlc    = 3,        // data byte count, see DL comments
                .brs    = 0,        // 0: no bit rate switching, 1: yes brs
                .fdf    = 0,        // FD format, 0: classic CAN, 1: CAN FD format
                .efc    = 0,        // 0: dont store Tx events, 1: store
                .mm     = 0x3,      // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
            };

        txmsg.data[0] = 6;
        txmsg.data[1] = 7;
        txmsg.data[2] = 8;

        DL_MCAN_TxFIFOStatus tf;
        DL_MCAN_getTxFIFOQueStatus(CANFD0, &tf);

        uint32_t bufferIndex = tf.putIdx;

        DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, bufferIndex, &txmsg);
        DL_MCAN_TXBufAddReq(CANFD0, tf.getIdx);

        vTaskDelay(pdMS_TO_TICKS(400));
        System::uart_ui.nputs(ARRANDN("meow" NEWLINE));
    };


//    PacketHeader pktHeader;
//    Pkt_CellV pktCellV;


    //pointer to variable -> point it to the start of the data array, no copy

    while (1)
    {
        // Slave CAN TX
        DL_MCAN_TxBufElement txmsg = {
                .id     = 0x1,      // CAN id, 11b->[28:18], 29b->[] when using 11b can id
                .rtr    = 0,        // 0: data frame, 1: remote frame
                .xtd    = 1,        // 0: 11b id, 1: 29b id
                .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
                .dlc    = 3,        // data byte count, see DL comments
                .brs    = 0,        // 0: no bit rate switching, 1: yes brs
                .fdf    = 1,        // FD format, 0: classic CAN, 1: CAN FD format
                .efc    = 0,        // 0: dont store Tx events, 1: store
                .mm     = 0x3,      // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
            };


        BMSComms::PacketHeader * header = static_cast<BMSComms::PacketHeader *>((void *)txmsg.data);
        BMSComms::PktSM_CellV * cellcv = static_cast<BMSComms::PktSM_CellV *>((void *)header->data);

        txmsg.data[0] = 6;
        txmsg.data[1] = 7;
        txmsg.data[2] = 8;

        DL_MCAN_TxFIFOStatus tf;
        DL_MCAN_getTxFIFOQueStatus(CANFD0, &tf);

        uint32_t bufferIndex = tf.putIdx;

        DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, bufferIndex, &txmsg);
        DL_MCAN_TXBufAddReq(CANFD0, tf.getIdx);

        vTaskDelay(pdMS_TO_TICKS(400));
        System::uart_ui.nputs(ARRANDN("meow" NEWLINE));
    }



    System::FailHard("BMS_task ended" NEWLINE);
    vTaskDelete(NULL);
}

bool setup_BBQ(BQ76952 & b){
    b.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::BQ769x2_RESET);
    vTaskDelay(pdMS_TO_TICKS(61));

    b.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::SET_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(9));

    // After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
    // programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
    // An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
    // a full description of the register and the bits will pop up on the screen.

    // 'Comm Type' - SPI with CRC
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::CommType, 0x10, 1);

    // 'Power Config' - 0x9234 = 0x2D80
    // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
    // Set wake speed bits to 00 for best performance
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::PowerConfig, 0x2D80, 2);

    // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::REG12Config, 0xFD, 1);

    // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::REG0Config, 0x01, 1);

    // Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::DFETOFFPinConfig, 0x42, 1);

    // Set up ALERT Pin - 0x92FC = 0x2A
    // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
    // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::ALERTPinConfig, 0x2A, 1);

    // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::TS1Config, 0x07, 1);

    // Set TS3 to measure FET Temperature - 0x92FF = 0x0F
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::TS3Config, 0x0F, 1);

    // Set HDQ to measure Cell Temperature - 0x9300 = 0x07
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::HDQPinConfig, 0x00, 1);  // No thermistor installed on EVM HDQ pin, so set to 0x00

    // 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
    // Only for openwire detection and  protection
    uint16_t u16TempValue = 0;
    for (uint8_t u8Count = 0; u8Count < (16 - 1); u8Count++) {
    u16TempValue += (0x1 << u8Count);
    }
    u16TempValue += 0x8000;
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::VCellMode, u16TempValue, 2);

    // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
    // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
    // COV (over-voltage), CUV (under-voltage)
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledProtectionsA, 0xBC, 1);

    // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
    // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledProtectionsB, 0xF7, 1);

    // 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::DefaultAlarmMask, 0xF882, 2);

    // Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
    // Also see "Cell Balancing with BQ769x2 Battery Monitors" document on ti.com
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::BalancingConfiguration, 0x03, 1);

    //Set the minimum cell balance voltage in charge - 0x933B = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinCellVCharge, 3.3 - 100, 2);
    //        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);
    //Set the minimum cell balance voltage in rest - 0x933F = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinCellVRelax, 3.3 - 100, 2);
    //        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);

    // Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
    // CUV Threshold is this value multiplied by 50.6mV
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::CUVThreshold, 0x31, 1);
    //    BQ769x2_SetRegister(
    //        CUVThreshold, pBattParamsCfg->u16MinBattVoltThd_mV / 51, 1);

    // Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
    // COV Threshold is this value multiplied by 50.6mV
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::COVThreshold, 0x55, 1);
    //    BQ769x2_SetRegister(
    //        COVThreshold, pBattParamsCfg->u16MaxBattVoltThd_mV / 51, 1);

    // Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::OCCThreshold, 0x05, 1);
    //    BQ769x2_SetRegister(
    //        OCCThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up OCD1 (over-current in discharge) Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::OCD1Threshold, 0x0A, 1);
    //    BQ769x2_SetRegister(
    //        OCD1Threshold, pBattParamsCfg->i16MinDhgCurtThd_mA / 2000, 1);

    // Set up SCD (short discharge current) Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  0x05=100mV
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::SCDThreshold, 0x05, 1);
    //    BQ769x2_SetRegister(
    //        SCDThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 us; min value of 1
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::SCDDelay, 0x03, 1);

    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::SCDLLatchLimit, 0x01, 1);


    vTaskDelay(pdMS_TO_TICKS(9));
    // Exit CONFIGUPDATE mode  - Subcommand 0x0092
    b.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::EXIT_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(9));
    //Control All FETs on
    b.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::FET_ENABLE);
    vTaskDelay(pdMS_TO_TICKS(9));
    b.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::ALL_FETS_ON);
    vTaskDelay(pdMS_TO_TICKS(9));
    b.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::SLEEP_DISABLE);
    vTaskDelay(pdMS_TO_TICKS(9));

    return true;
}
