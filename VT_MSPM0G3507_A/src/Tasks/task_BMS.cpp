/*
 * task_DAQ.cpp
 *
 *  Created on: Oct 4, 2025
 *      Author: turtl
 */

#include <stdio.h>

#include <Tasks/task_BMS.hpp>
#include "Core/system.hpp"
#include "Middleware/BQ769x2/BQ76952.hpp"
#include <array>

/** configures the settings on the thingy
 *  @return true on success
 */
bool setup_BBQ(BQ76952 &);

void Task::BMS_task(void *){
    BQ76952 bq;
    bq.i2c_controller   = &System::i2c1;
    bq.i2c_addr         = 0x8;

    bq.i2c_controller->setSCLTarget(50e3);

    { // SEAL -> UNSEAL
        //bqTM.13.8.2/197

        /* bqTM.8.1/71
         *  """
         *      each transition requires that a unique set of keys be sent to the device
         *      through the sub-command address (0x3E and 0x3F). The keys must be sent
         *      consecutively to 0x3E and 0x3F, with no other data written between the
         *      keys. Do not set the two keys to identical values
         *  """
         */

        uint32_t usk = 0x3672'0414; // un-seal key
//        uint32_t usk = 0x3672'0414; // un-seal key : factory default

        uint8_t sk1[] = {0x3E, (uint8_t)((usk & 0x0000'00FF) >> 00), (uint8_t)((usk & 0x0000'FF00) >> 8)};
        bq.i2c_controller->tx_blocking(bq.i2c_addr, sk1, sizeof(sk1), pdMS_TO_TICKS(10));

        uint8_t sk2[] = {0x3E, (uint8_t)((usk & 0x00FF'0000) >> 16), (uint8_t)((usk & 0xFF00'0000) >> 24)};
        bq.i2c_controller->tx_blocking(bq.i2c_addr, sk2, sizeof(sk2), pdMS_TO_TICKS(10));
    }

    while(! setup_BBQ(bq)){
        System::uart_ui.nputs(ARRANDN("failed to init BBQ" NEWLINE));
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



    const BQ769X2_PROTOCOL::CmdDrt cmds[] = {
//             BQ769X2_PROTOCOL::CmdDrt::StackVoltage,
//             BQ769X2_PROTOCOL::CmdDrt::IntTemperature, // units of 0.1k
             BQ769X2_PROTOCOL::CmdDrt::Cell1Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell2Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell3Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell4Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell5Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell6Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell7Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell8Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell9Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell10Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell11Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell12Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell13Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell14Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell15Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell16Voltage
        };

    char str[MAX_STR_LEN_COMMON];

    while(1){
        uint32_t v= 0xBeeF;
        bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::PowerConfig, &v, sizeof(v));
        if((0x2D80 | BV(6)) != v){
            char str[20];
            snprintf(ARRANDN(str), "%04h", v);
            System::uart_ui.nputs(ARRANDN(str));
            System::uart_ui.nputs(ARRANDN("meow" NEWLINE));
        } else
            break;
    }

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

        vTaskDelay(pdMS_TO_TICKS(100));
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
    System::FailHard("BMS_task ended" NEWLINE);
    vTaskDelete(NULL);
}

bool setup_BBQ(BQ76952 & bq){
    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::BQ769x2_RESET);
    vTaskDelay(pdMS_TO_TICKS(61));

    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::SET_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(9));

    // After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
    // programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
    // An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
    // a full description of the register and the bits will pop up on the screen.

    // 'Comm Type' - I2C without CRC for TX only, RX will still have to deal with it
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::CommType, 0x07, 1);

    // 'Power Config' - 0x9234 = 0x2D80
    // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
    // Set wake speed bits to 00 for best performance
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::PowerConfig, 0x2D80 | BV(6), 2);

    // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::REG0Config, 0x01, 1);

    // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::REG12Config, 0x0D, 1);

    // Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::DFETOFFPinConfig, 0x42, 1);

    // Set up ALERT Pin - 0x92FC = 0x2A
    // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
    // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::ALERTPinConfig, 0x2A, 1);

    // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::TS1Config, 0x07, 1);

    // Set TS3 to measure FET Temperature - 0x92FF = 0x0F
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::TS3Config, 0x0F, 1);

    // Set HDQ to measure Cell Temperature - 0x9300 = 0x07
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::HDQPinConfig, 0x00, 1);  // No thermistor installed on EVM HDQ pin, so set to 0x00

    // 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
    // Only for openwire detection and  protection
    uint16_t u16TempValue = 0;
    for (uint8_t u8Count = 0; u8Count < (16 - 1); u8Count++) {
    u16TempValue += (0x1 << u8Count);
    }
    u16TempValue += 0x8000;
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::VCellMode, u16TempValue, 2);

    // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
    // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
    // COV (over-voltage), CUV (under-voltage)
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledProtectionsA, 0xBC, 1);

    // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
    // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledProtectionsB, 0xF7, 1);

    // 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::DefaultAlarmMask, 0xF882, 2);

    // Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
    // Also see "Cell Balancing with BQ769x2 Battery Monitors" document on ti.com
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::BalancingConfiguration, 0x03, 1);

    //Set the minimum cell balance voltage in charge - 0x933B = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinCellVCharge, 3.3 - 100, 2);
    //        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);
    //Set the minimum cell balance voltage in rest - 0x933F = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinCellVRelax, 3.3 - 100, 2);
    //        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);

    // Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
    // CUV Threshold is this value multiplied by 50.6mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::CUVThreshold, 0x31, 1);
    //    BQ769x2_SetRegister(
    //        CUVThreshold, pBattParamsCfg->u16MinBattVoltThd_mV / 51, 1);

    // Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
    // COV Threshold is this value multiplied by 50.6mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::COVThreshold, 0x55, 1);
    //    BQ769x2_SetRegister(
    //        COVThreshold, pBattParamsCfg->u16MaxBattVoltThd_mV / 51, 1);

    // Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::OCCThreshold, 0x05, 1);
    //    BQ769x2_SetRegister(
    //        OCCThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up OCD1 (over-current in discharge) Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::OCD1Threshold, 0x0A, 1);
    //    BQ769x2_SetRegister(
    //        OCD1Threshold, pBattParamsCfg->i16MinDhgCurtThd_mA / 2000, 1);

    // Set up SCD (short discharge current) Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  0x05=100mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::SCDThreshold, 0x05, 1);
    //    BQ769x2_SetRegister(
    //        SCDThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 us; min value of 1
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::SCDDelay, 0x03, 1);

    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::SCDLLatchLimit, 0x01, 1);


    vTaskDelay(pdMS_TO_TICKS(9));
    // Exit CONFIGUPDATE mode  - Subcommand 0x0092
    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::EXIT_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(9));
    //Control All FETs on
    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::FET_ENABLE);
    vTaskDelay(pdMS_TO_TICKS(9));
    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::ALL_FETS_ON);
    vTaskDelay(pdMS_TO_TICKS(9));
    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::SLEEP_DISABLE);
    vTaskDelay(pdMS_TO_TICKS(9));

    return true;
}
