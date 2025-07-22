/*
 * BQ796x2_PROTOCOL_Test_V.cpp
 *
 *  Created on: Jul 13, 2025
 *      Author: FSAE
 */

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <ti/driverlib/driverlib.h>
#include <Tasks/BQ769x2_PROTOCOL_Test_V.hpp>

#include "Middleware/BQ769x2_PROTOCOL.hpp"
#include "Core/system.hpp"

void Task::BQ769x2_PROTOCOL_Test_V_Task(void*) {
    System::uart_ui.nputs(ARRANDN("BQ769x2_PROTOCOL_Test_V_Task Start" NEWLINE));

    char str[MAX_STR_LEN_COMMON];


    //--- communicaiton -----------------------------------------

    BQ769X2_PROTOCOL::sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::BQ769x2_RESET);
    vTaskDelay(pdMS_TO_TICKS(60));
    BQ769X2_PROTOCOL::sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::SET_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(8));


    // After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
    // programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
    // An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
    // a full description of the register and the bits will pop up on the screen.

    // 'Power Config' - 0x9234 = 0x2D80
    // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
    // Set wake speed bits to 00 for best performance
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::PowerConfig, 0x2D80, 2);

    // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::REG0Config, 0x01, 1);

    // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::REG12Config, 0x0D, 1);

    // Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::DFETOFFPinConfig, 0x42, 1);

    // Set up ALERT Pin - 0x92FC = 0x2A
    // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
    // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::ALERTPinConfig, 0x2A, 1);

    // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::TS1Config, 0x07, 1);

    // Set TS3 to measure FET Temperature - 0x92FF = 0x0F
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::TS3Config, 0x0F, 1);

    // Set HDQ to measure Cell Temperature - 0x9300 = 0x07
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::HDQPinConfig, 0x00, 1);  // No thermistor installed on EVM HDQ pin, so set to 0x00

    // 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
    // Only for openwire detection and  protection
    uint16_t u16TempValue = 0;
    for (uint8_t u8Count = 0; u8Count < (16 - 1); u8Count++) {
        u16TempValue += (0x1 << u8Count);
    }
    u16TempValue += 0x8000;
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::VCellMode, u16TempValue, 2);

    // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
    // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
    // COV (over-voltage), CUV (under-voltage)
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledProtectionsA, 0xBC, 1);

    // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
    // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledProtectionsB, 0xF7, 1);

    // 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::DefaultAlarmMask, 0xF882, 2);

    // Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
    // Also see "Cell Balancing with BQ769x2 Battery Monitors" document on ti.com
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::BalancingConfiguration, 0x03, 1);

    //Set the minimum cell balance voltage in charge - 0x933B = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinCellVCharge, 3.3 - 100, 2);
//        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);
    //Set the minimum cell balance voltage in rest - 0x933F = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinCellVRelax, 3.3 - 100, 2);
//        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);

    // Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
    // CUV Threshold is this value multiplied by 50.6mV
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::CUVThreshold, 0x31, 1);
//    BQ769x2_SetRegister(
//        CUVThreshold, pBattParamsCfg->u16MinBattVoltThd_mV / 51, 1);

    // Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
    // COV Threshold is this value multiplied by 50.6mV
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::COVThreshold, 0x55, 1);
//    BQ769x2_SetRegister(
//        COVThreshold, pBattParamsCfg->u16MaxBattVoltThd_mV / 51, 1);

    // Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::OCCThreshold, 0x05, 1);
//    BQ769x2_SetRegister(
//        OCCThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up OCD1 (over-current in discharge) Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::OCD1Threshold, 0x0A, 1);
//    BQ769x2_SetRegister(
//        OCD1Threshold, pBattParamsCfg->i16MinDhgCurtThd_mA / 2000, 1);

    // Set up SCD (short discharge current) Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  0x05=100mV
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::SCDThreshold, 0x05, 1);
//    BQ769x2_SetRegister(
//        SCDThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 us; min value of 1
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::SCDDelay, 0x03, 1);

    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    BQ769X2_PROTOCOL::setRegister(BQ769X2_PROTOCOL::RegAddr::SCDLLatchLimit, 0x01, 1);


    vTaskDelay(pdMS_TO_TICKS(8));
    // Exit CONFIGUPDATE mode  - Subcommand 0x0092
    BQ769X2_PROTOCOL::sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::EXIT_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(8));
    //Control All FETs on
    BQ769X2_PROTOCOL::sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::FET_ENABLE);
    vTaskDelay(pdMS_TO_TICKS(8));
    BQ769X2_PROTOCOL::sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::ALL_FETS_ON);
    vTaskDelay(pdMS_TO_TICKS(8));
    BQ769X2_PROTOCOL::sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::SLEEP_DISABLE);
    vTaskDelay(pdMS_TO_TICKS(8));

//    System::i2c1.tx_ctrl_blocking(0x08, ARRANDN(((uint8_t[]){0x36, 0x72, 0x41, 0x4})));
    const BQ769X2_PROTOCOL::CmdDrt cmds[16] = {
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
             BQ769X2_PROTOCOL::CmdDrt::Cell12Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell13Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell14Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell15Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell16Voltage
        };

    while(true){
        for(uint8_t i = 0; i < sizeof(cmds); i++){
            uint16_t v = 0;
            BQ769X2_PROTOCOL::I2C_ReadReg(cmds[i], (uint8_t *)&v, BQ769X2_PROTOCOL::DIR_CMD_TYPE::W2);

            snprintf(ARRANDN(str), "%d,", v);
            System::uart_ui.nputs(ARRANDN(str));
//            vTaskDelay(pdMS_TO_TICKS(10));
        }
        System::uart_ui.nputs(ARRANDN(NEWLINE));
    }

    System::uart_ui.nputs(ARRANDN("fiddle task end" NEWLINE));
    vTaskDelete(NULL);
}
