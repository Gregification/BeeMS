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


// user amp to centiamp scale (10mA)
// using 10mA scale (determined by DAConfiguration)
#define userAto10mA(X) (X)

// user volt to centivolt scale (10mV)
// using 10mV scale (determined by DAConfiguration)
#define userVto10mV(X) (X)

// only 1 BQ on the voltage tap board
BQ76952 bq = {
        .spi  = &System::spi1,
        .cs   = &System::GPIO::PA15,
    };

BQ76952::BQ76952SSetting constexpr bqSetting = {
        .Fuse = {
            .minBlowFuseVoltage_10mV= 75000 / 10,   // 75V
            .timeout_S              = 0,            // 0:indefinite
        },
        .Configuration = {
            .powerConfig = {
                .wake_speed     = 3,
                .loop_slow      = 0,
                .cb_loop_slow   = 3,
                .fastadc        = 0,
                .otsd           = 1,
                .sleep          = 0,
                .dpslp_lfo      = 0,
                .dpslp_ldo      = 1,
                .dpslp_pd       = 1,
                .shut_ts2       = 1,
                .dpslp_ot       = 1,
            },
            .REG0Config = {
                .enable0    = 1,
            },
            .HWDRegulatorOptions = { // TODO: is a safety thing, set up properly on final product
                .toggle_time    = 10, // 5S power cycle
                .toggle_opt     = 2,  // turn-off then turn-on on HWD
            },
            .spiConfig = {
                .filt       = 0, // no digital filter
                .miso_reg1  = 1, // logic high V = REG1 output
            },
            .commIdleTime_S = 1, // 1S of no comms before turning off HFO
            .cfetoffPinConfig = {// WARNING: effects SPI CS behavior
                .Raw = 0,   // as SPI cs
            },
            .dfetoffPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                .function   = 3,    // ADC input / thermistor
                .opt1_0     = 0b00, // as ADC
                .opt3_2     = 0b11, // no polynomial, report raw ADC
                .opt5_4     = 0b10, // no PU
            },
            .alertPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                .function   = 3,    // ADC input / thermistor
                .opt1_0     = 0b00, // as ADC
                .opt3_2     = 0b11, // no polynomial, report raw ADC
                .opt5_4     = 0b10, // no PU
            },
            .TS1Config = { // TODO: figure out how to do custom thermistor polynomial
                .function   = 3,    // ADC input / thermistor
                .opt1_0     = 0b00, // as ADC
                .opt3_2     = 0b11, // no polynomial, report raw ADC
                .opt5_4     = 0b10, // no PU
            },
            .TS2Config = { // TODO: figure out how to do custom thermistor polynomial
                .function   = 3,    // ADC input / thermistor
                .opt1_0     = 0b00, // as ADC
                .opt3_2     = 0b11, // no polynomial, report raw ADC
                .opt5_4     = 0b10, // no PU
            },
            .TS3Config = { // TODO: figure out how to do custom thermistor polynomial
                .function   = 3,    // ADC input / thermistor
                .opt1_0     = 0b00, // as ADC
                .opt3_2     = 0b11, // no polynomial, report raw ADC
                .opt5_4     = 0b10, // no PU
            },
            .HDQPinConfig = { // WARNING: effects SPI MOSI behavior
                .Raw = 0,   // as SPI MOSI
            },
            .DCHGPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                .function   = 3,    // ADC input / thermistor
                .opt1_0     = 0b00, // as ADC
                .opt3_2     = 0b11, // no polynomial, report raw ADC
                .opt5_4     = 0b10, // no PU
            },
            .DDSGPinConfig = { // TODO: figure out how to do custom thermistor polynomial
                .function   = 3,    // ADC input / thermistor
                .opt1_0     = 0b00, // as ADC
                .opt3_2     = 0b11, // no polynomial, report raw ADC
                .opt5_4     = 0b10, // no PU
            },
            .DAConfig = {
                .user_amps  = 2, // USER AMPS unit selection. 2:10mA,3:100mA . see userAto10mA macro
                .user_volts = 1, // USER VOLTS unit selection. 0:1mV,1:10mV . see userVto10mV macro
                .tint_en    = 0, // die temp used as a cell temp? 0:no, 1:yes.
                .tint_fett  = 0, // die tmep used as fet temp? 0:no, 1:yes
            },
            .VcellMode  = 0x420,
            .CC3Samples = 0x0F,
        },
    };

void Task::BMS_task(void *){
    System::uart_ui.nputs(ARRANDN("BMS_task start" NEWLINE));

    //---- SPI setup ------------------------------------------
    bq.spi->setSCLKTarget(250e3); // bq76952 max speed of 2MHz
    DL_GPIO_enableOutput(GPIOPINPUX(*bq.cs));
    DL_GPIO_initDigitalOutputFeatures(
            bq.cs->iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    bq.cs->clear();

    vTaskDelay(pdMS_TO_TICKS(50)); // CS needs some time to get recognized by the slave

    //--- init BQ76952 ----------------------------------------
    // TODO: on the final product we need to somehow prevent the MCU from locking up the BQ,
    //      if the MCU gets in a power cycle loop.

//    bq.unseal(0x36720414);

//    {
//        uint16_t batt_status;
//        if(bq.sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt::BatteryStatus, PTRANDN(batt_status)))
//        switch((batt_status >> 8) & 0b11){
//            case 0: System::uart_ui.nputs(ARRANDN("not initialized")); break;
//            case 1: System::uart_ui.nputs(ARRANDN("full access")); break;
//            case 2: System::uart_ui.nputs(ARRANDN("unsealed")); break;
//            case 3: System::uart_ui.nputs(ARRANDN("sealed")); break;
//        }
//        else System::uart_ui.nputs(ARRANDN(":("));
//        System::uart_ui.nputs(ARRANDN(NEWLINE));
//    }

    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::BQ769x2_RESET);
    vTaskDelay(pdMS_TO_TICKS(61));

    if(! bq.setConfig(&bqSetting))
        System::FailHard("failed to init BBQ settings on MCU power up. failed to write");

    {
        BQ76952::BQ76952SSetting read;
        if(!bq.getConfig(&read))
            System::FailHard("failed to init BBQ settings on MCU power up. failed to read");

//        if(!(read == bqSetting))
//            System::FailHard("failed to init BBQ settings on MCU power up. READ != WRITE");
    }

    while(1){
        struct {
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
        } v;
        bool success = bq.sendSubcommandR(BQ769X2_PROTOCOL::Cmd::DASTATUS5, &v, sizeof(v));

        // Print Header
        System::uart_ui.nputs(ARRANDN("--- Battery Data Block ---" NEWLINE));

        // 1. VREG18 (16-bit)
        System::uart_ui.nputs(ARRANDN("VREG18 (ADC counts): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.Vreg18));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 2. VSS (16-bit)
        System::uart_ui.nputs(ARRANDN("VSS (ADC counts): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.VSS));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 3. Max Cell Voltage (16-bit)
        System::uart_ui.nputs(ARRANDN("Max Cell Voltage (mV): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.MaxCellVoltage));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 4. Min Cell Voltage (16-bit)
        System::uart_ui.nputs(ARRANDN("Min Cell Voltage (mV): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.MinCellVoltage));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 5. Battery Voltage Sum (16-bit)
        System::uart_ui.nputs(ARRANDN("Battery Voltage Sum (cV): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.BatteryVoltageSum));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 6. Avg Cell Temperature (16-bit)
        System::uart_ui.nputs(ARRANDN("Avg Cell Temperature (0.1 K): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.AvgCellTemperature));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 7. FET Temperature (16-bit)
        System::uart_ui.nputs(ARRANDN("FET Temperature (0.1 K): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.FETTemperature));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 8. Max Cell Temperature (16-bit)
        System::uart_ui.nputs(ARRANDN("Max Cell Temperature (0.1 K): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.MaxCellTemperature));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 9. Min Cell Temperature (16-bit)
        System::uart_ui.nputs(ARRANDN("Min Cell Temperature (0.1 K): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.MinCellTemperature));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 10. Second Avg Cell Temperature (16-bit)
        System::uart_ui.nputs(ARRANDN("Avg Cell Temp 2 (0.1 K): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.SecondAvgCellTemperature));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 11. CC3 Current (16-bit)
        System::uart_ui.nputs(ARRANDN("CC3 Current (userA): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.CC3Current));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 12. CC1 Current (16-bit)
        System::uart_ui.nputs(ARRANDN("CC1 Current (userA): "));
        System::uart_ui.putu32d(static_cast<uint32_t>(v.CC1Current));
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 13. CC2 Counts (32-bit)
        System::uart_ui.nputs(ARRANDN("CC2 Counts (ADC counts): "));
        System::uart_ui.putu32d(v.CC2Counts);
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        // 14. CC3 Counts (32-bit)
        System::uart_ui.nputs(ARRANDN("CC3 Counts (ADC counts): "));
        System::uart_ui.putu32d(v.CC3Counts);
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        if(!success)
            System::uart_ui.nputs(ARRANDN(CLIBAD "womp" NEWLINE CLIRESET));

        // Cell Balancing
        uint16_t cb_ac = 0;
        char str[128];

        bool cb_wr = bq.sendSubcommandW2(BQ769X2_PROTOCOL::Cmd::CB_ACTIVE_CELLS, 0x1); //starts balancing on cell 0

        bool cb_rd = bq.sendSubcommandR(BQ769X2_PROTOCOL::Cmd::CB_ACTIVE_CELLS, &cb_ac, sizeof(cb_ac));

        if(cb_wr && cb_rd)
        {
            snprintf(ARRANDN(str), "CB Active Cells: %x,", cb_ac);
            System::uart_ui.nputs(ARRANDN(str));
            System::uart_ui.nputs(ARRANDN(NEWLINE));
        }
        else
            System::uart_ui.nputs(ARRANDN(CLIBAD "panik 2" NEWLINE CLIRESET));





        //fatal error 0 : failed to init BBQ settings on MCU power up. failed to write ...
        // Read Internal Temperature using 0x68 command
        uint16_t internalTempRaw = 0;
        bool tempSuccess = bq.sendDirectCommandR(
            BQ769X2_PROTOCOL::CmdDrt::IntTemperature,  // 0x68
            &internalTempRaw,
            sizeof(internalTempRaw)
        );

        // Print Internal Temperature
        System::uart_ui.nputs(ARRANDN("--- Internal Temperature ---" NEWLINE));

        if(tempSuccess) {
            // Internal temperature is already in units of 0.1 K from the BQ chip
            // The chip handles the calibration formula internally:
            // Temp = (ADC × Int Gain / 65536) + Int base offset + Int Temp Offset

            System::uart_ui.nputs(ARRANDN("Internal Temp (0.1 K): "));
            System::uart_ui.putu32d(static_cast<uint32_t>(internalTempRaw));
            System::uart_ui.nputs(ARRANDN(NEWLINE));

            // 0.1 K units, so divide by 10 to get K, then subtract 273.15 for °C
            int32_t tempCelsius = (static_cast<int32_t>(internalTempRaw) - 2731) / 10;
            System::uart_ui.nputs(ARRANDN("Internal Temp (C): "));
            if(tempCelsius < 0) {
                System::uart_ui.nputs(ARRANDN("-"));
                tempCelsius = -tempCelsius;
            }
            System::uart_ui.putu32d(static_cast<uint32_t>(tempCelsius));
            System::uart_ui.nputs(ARRANDN(NEWLINE));
        } else {
            System::uart_ui.nputs(ARRANDN(CLIBAD "Failed to read internal temperature" NEWLINE CLIRESET));
        }

        System::uart_ui.nputs(ARRANDN(NEWLINE));

        vTaskDelay(pdMS_TO_TICKS(1e3));
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

/*
    DL_MCAN_TxBufElement txmsg = {
           .id     = 0x1,      // CAN id, 11b->[28:18], 29b->[] when using 11b can id
           .rtr    = 0,        // 0: data frame, 1: remote frame
           .xtd    = 1,        // 0: 11b id, 1: 29b id
           .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
           .dlc    = 8,        // data byte count, see DL comments
           .brs    = 0,        // 0: no bit rate switching, 1: yes brs
           .fdf    = 0,        // FD format, 0: classic CAN, 1: CAN FD format
           .efc    = 0,        // 0: dont store Tx events, 1: store
           .mm     = 0x3,      // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
       };

    while(1){
        for(uint8_t i = 0; i < sizeof(cmds); i++){
            uint16_t v = 0xBEEF;
            bool success = bq.sendDirectCommandR(cmds[i], &v);
            vTaskDelay(pdMS_TO_TICKS(1e6));

            snprintf(ARRANDN(str), "%6d,", v);

            System::uart_ui.nputs(ARRANDN(str));

            if(!success)
                v = 0;

            ((uint16_t*)txmsg.data)[i % 8] = v;

            // transmit in 8B packets
            if((i+1) % 8 == 0)
            {
               DL_MCAN_TxFIFOStatus tf;
               DL_MCAN_getTxFIFOQueStatus(CANFD0, &tf);

               uint32_t bufferIndex = tf.putIdx;

               DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, bufferIndex, &txmsg);
               DL_MCAN_TXBufAddReq(CANFD0, tf.getIdx);
            }
        }
        System::uart_ui.nputs(ARRANDN(NEWLINE));

        vTaskDelay(pdMS_TO_TICKS(1e6));
    }




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



//        txmsg.data[0] = 6;
//        txmsg.data[1] = 7;
//        txmsg.data[2] = 8;

        DL_MCAN_TxFIFOStatus tf;
        DL_MCAN_getTxFIFOQueStatus(CANFD0, &tf);

        uint32_t bufferIndex = tf.putIdx;

        DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, bufferIndex, &txmsg);
        DL_MCAN_TXBufAddReq(CANFD0, tf.getIdx);

        vTaskDelay(pdMS_TO_TICKS(400));
    };
    */


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

    // 'Comm Type' - I2C without CRC for TX only, RX will still have to deal with it
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::CommType, 0x07, 1);

    // 'Power Config' - 0x9234 = 0x2D80
    // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
    // Set wake speed bits to 00 for best performance
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::PowerConfig, 0x2D80 | BV(6), 2);

    // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::REG0Config, 0x01, 1);

    // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
    b.setRegister(BQ769X2_PROTOCOL::RegAddr::REG12Config, 0x0D, 1);

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
