/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

/*
 * TODO: look at later : https://www.tij.co.jp/jp/lit/ml/slyp847/slyp847.pdf?ts=1749927951492&ref_url=https%253A%252F%252Fwww.google.com%252F
 */

#include "fiddle.hpp"

#include <FreeRTOS.h>
#include <Middleware/BQ769x2_PROTOCOL.hpp>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <stdio.h>

#include "system.hpp"

//void fiddle_task(void *){
//
//    vTaskDelete(NULL);
//}

//I2C
void Task::fiddle_task(void *){
    System::uart_ui.nputs(ARRANDN("fiddle task start" NEWLINE));

    DL_I2C_enablePower(System::i2c1.reg);
    delay_cycles(POWER_STARTUP_DELAY);

    // PA15
    DL_GPIO_initPeripheralInputFunctionFeatures(
            IOMUX_PINCM37,
            IOMUX_PINCM37_PF_I2C1_SCL,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_DISABLE,
            DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
        );
    // PA16
    DL_GPIO_initPeripheralInputFunctionFeatures(
            IOMUX_PINCM38,
            IOMUX_PINCM38_PF_I2C1_SDA,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_DISABLE,
            DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
        );
    DL_GPIO_enableHiZ(IOMUX_PINCM37);
    DL_GPIO_enableHiZ(IOMUX_PINCM38);
    System::i2c1.partialInitController();
    System::i2c1.setSCLTarget(100e3);
    DL_I2C_enableController(System::i2c1.reg);

    char str[MAX_STR_LEN_COMMON];
    vTaskDelay(pdMS_TO_TICKS(500));

    BQ769x2 bbq;

    bbq.sendCommandSubcommand(BQ769x2::BQ769x2_RESET);
    bbq.delayUS(60000);
    bbq.sendCommandSubcommand(BQ769x2::SET_CFGUPDATE);
    bbq.delayUS(8000);


    // After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
    // programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
    // An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
    // a full description of the register and the bits will pop up on the screen.

    // 'Power Config' - 0x9234 = 0x2D80
    // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
    // Set wake speed bits to 00 for best performance
    bbq.setRegister(BQ769x2::PowerConfig, 0x2D80, 2);

    // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
    bbq.setRegister(BQ769x2::REG0Config, 0x01, 1);

    // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
    bbq.setRegister(BQ769x2::REG12Config, 0x0D, 1);

    // Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
    bbq.setRegister(BQ769x2::DFETOFFPinConfig, 0x42, 1);

    // Set up ALERT Pin - 0x92FC = 0x2A
    // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
    // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
    bbq.setRegister(BQ769x2::ALERTPinConfig, 0x2A, 1);

    // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
    bbq.setRegister(BQ769x2::TS1Config, 0x07, 1);

    // Set TS3 to measure FET Temperature - 0x92FF = 0x0F
    bbq.setRegister(BQ769x2::TS3Config, 0x0F, 1);

    // Set HDQ to measure Cell Temperature - 0x9300 = 0x07
    bbq.setRegister(BQ769x2::HDQPinConfig, 0x00, 1);  // No thermistor installed on EVM HDQ pin, so set to 0x00

    // 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
    // Only for openwire detection and  protection
    uint16_t u16TempValue = 0;
    for (uint8_t u8Count = 0; u8Count < (16 - 1); u8Count++) {
        u16TempValue += (0x1 << u8Count);
    }
    u16TempValue += 0x8000;
    bbq.setRegister(BQ769x2::VCellMode, u16TempValue, 2);

    // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
    // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
    // COV (over-voltage), CUV (under-voltage)
    bbq.setRegister(BQ769x2::EnabledProtectionsA, 0xBC, 1);

    // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
    // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
    bbq.setRegister(BQ769x2::EnabledProtectionsB, 0xF7, 1);

    // 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
    bbq.setRegister(BQ769x2::DefaultAlarmMask, 0xF882, 2);

    // Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
    // Also see "Cell Balancing with BQ769x2 Battery Monitors" document on ti.com
    bbq.setRegister(BQ769x2::BalancingConfiguration, 0x03, 1);

    //Set the minimum cell balance voltage in charge - 0x933B = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    bbq.setRegister(BQ769x2::CellBalanceMinCellVCharge, 3.3 - 100, 2);
//        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);
    //Set the minimum cell balance voltage in rest - 0x933F = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    bbq.setRegister(BQ769x2::CellBalanceMinCellVRelax, 3.3 - 100, 2);
//        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);

    // Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
    // CUV Threshold is this value multiplied by 50.6mV
    bbq.setRegister(BQ769x2::CUVThreshold, 0x31, 1);
//    BQ769x2_SetRegister(
//        CUVThreshold, pBattParamsCfg->u16MinBattVoltThd_mV / 51, 1);

    // Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
    // COV Threshold is this value multiplied by 50.6mV
    bbq.setRegister(BQ769x2::COVThreshold, 0x55, 1);
//    BQ769x2_SetRegister(
//        COVThreshold, pBattParamsCfg->u16MaxBattVoltThd_mV / 51, 1);

    // Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
    bbq.setRegister(BQ769x2::OCCThreshold, 0x05, 1);
//    BQ769x2_SetRegister(
//        OCCThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up OCD1 (over-current in discharge) Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
    bbq.setRegister(BQ769x2::OCD1Threshold, 0x0A, 1);
//    BQ769x2_SetRegister(
//        OCD1Threshold, pBattParamsCfg->i16MinDhgCurtThd_mA / 2000, 1);

    // Set up SCD (short discharge current) Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  0x05=100mV
    bbq.setRegister(BQ769x2::SCDThreshold, 0x05, 1);
//    BQ769x2_SetRegister(
//        SCDThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 us; min value of 1
    bbq.setRegister(BQ769x2::SCDDelay, 0x03, 1);

    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    bbq.setRegister(BQ769x2::SCDLLatchLimit, 0x01, 1);


    bbq.delayUS(8000);
    // Exit CONFIGUPDATE mode  - Subcommand 0x0092
    bbq.sendCommandSubcommand(BQ769x2::EXIT_CFGUPDATE);
    bbq.delayUS(8000);
    //Control All FETs on
    bbq.sendCommandSubcommand(BQ769x2::FET_ENABLE);
    bbq.delayUS(8000);
    bbq.sendCommandSubcommand(BQ769x2::ALL_FETS_ON);
    bbq.delayUS(8000);
    bbq.sendCommandSubcommand(BQ769x2::SLEEP_DISABLE);
    bbq.delayUS(8000);

//    System::i2c1.tx_ctrl_blocking(0x08, ARRANDN(((uint8_t[]){0x36, 0x72, 0x41, 0x4})));
    const BQ769x2::CmdDrt cmds[16] = {
             BQ769x2::CmdDrt::Cell1Voltage,
             BQ769x2::CmdDrt::Cell2Voltage,
             BQ769x2::CmdDrt::Cell3Voltage,
             BQ769x2::CmdDrt::Cell4Voltage,
             BQ769x2::CmdDrt::Cell5Voltage,
             BQ769x2::CmdDrt::Cell6Voltage,
             BQ769x2::CmdDrt::Cell7Voltage,
             BQ769x2::CmdDrt::Cell8Voltage,
             BQ769x2::CmdDrt::Cell9Voltage,
             BQ769x2::CmdDrt::Cell10Voltage,
             BQ769x2::CmdDrt::Cell12Voltage,
             BQ769x2::CmdDrt::Cell13Voltage,
             BQ769x2::CmdDrt::Cell14Voltage,
             BQ769x2::CmdDrt::Cell15Voltage,
             BQ769x2::CmdDrt::Cell16Voltage
        };

    while(true){
        for(uint8_t i = 0; i < sizeof(cmds); i++){
            uint16_t v;
            I2C_ReadReg(cmds[i], (uint8_t *)&v, W2);

            snprintf(ARRANDN(str), "%d,", v);
            System::uart_ui.nputs(ARRANDN(str));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        System::uart_ui.nputs(ARRANDN(NEWLINE));
    }

    System::uart_ui.nputs(ARRANDN("fiddle task end" NEWLINE));
    vTaskDelete(NULL);
}

// SPI
//void fiddle_task(void *){
//    // - SPI0 goes up to 32Mhz. speed restraints depending on MCU speed. check DOCs.
//
//    DL_SPI_enablePower(System::spi0.reg);
//    delay_cycles(POWER_STARTUP_DELAY);
//
//    /*--- GPIO config ----------------*/
//
//    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM43, IOMUX_PINCM43_PF_SPI0_PICO);// MOSI , PB17
//    DL_GPIO_initPeripheralInputFunctionFeatures(   // MISO , PB19
//            IOMUX_PINCM45,
//            IOMUX_PINCM45_PF_SPI0_POCI,
//            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
//            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
//            DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_DISABLE,
//            DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
//        );
//    DL_GPIO_enableHiZ(IOMUX_PINCM45);
//    DL_GPIO_initPeripheralOutputFunctionFeatures(   // SCLK , PB18
//            IOMUX_PINCM44,
//            IOMUX_PINCM44_PF_SPI0_SCLK,
//            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
//            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
//            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
//            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
//        );
//    DL_GPIO_initDigitalOutputFeatures(   // CS2  , PB20
//            IOMUX_PINCM48,
//            DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE,
//            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
//            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
//            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
//        );
//    DL_SPI_setChipSelect(System::spi0.reg, DL_SPI_CHIP_SELECT::DL_SPI_CHIP_SELECT_NONE);
//    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_17 | DL_GPIO_PIN_18 | DL_GPIO_PIN_19 | DL_GPIO_PIN_20);
//
//    System::GPIO::GPIO cs = {.port = GPIOB, .pin = DL_GPIO_PIN_20};
//    cs.clear();
//
//    vTaskDelay(pdMS_TO_TICKS(100));
//
//    /*--- SPI config -----------------*/
//
//    System::spi0.partialInit();
//    DL_SPI_Config config = {
//            .mode           = DL_SPI_MODE::DL_SPI_MODE_CONTROLLER,
//            .frameFormat    = DL_SPI_FRAME_FORMAT::DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0,
//            .parity         = DL_SPI_PARITY::DL_SPI_PARITY_NONE,
//            .dataSize       = DL_SPI_DATA_SIZE::DL_SPI_DATA_SIZE_8,
//            .bitOrder       = DL_SPI_BIT_ORDER::DL_SPI_BIT_ORDER_LSB_FIRST,
//            .chipSelectPin  = DL_SPI_CHIP_SELECT::DL_SPI_CHIP_SELECT_2,
//        };
//    DL_SPI_init(System::spi0.reg, &config);
//    DL_SPI_disablePacking(System::spi0.reg);
//    System::spi0.setSCLKTarget(125e3);
//    DL_SPI_enable(System::spi0.reg);
//
//    /*--- stuff ----------------------*/
//
//    uint8_t buff[10] = {0x22, BV(4)};
//
//    cs.set();
//    System::spi0.tx_blocking(buff, 3, &cs);
//    delay_cycles(80 * 100);
//    System::spi0.rx_blocking(buff, 3, &cs);
//
//    System::uart_ui.nputs(ARRANDN("end of fiddle" NEWLINE));
//
//    vTaskDelete(NULL);
//}

