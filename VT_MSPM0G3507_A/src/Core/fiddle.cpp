/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

#include "fiddle.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <stdio.h>

#include "system.hpp"

//void fiddle_task(void *){
//
//    vTaskDelete(NULL);
//}

//I2C
void fiddle_task(void *){

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
    DL_GPIO_disableHiZ(IOMUX_PINCM37);
    DL_GPIO_disableHiZ(IOMUX_PINCM38);
//    DL_GPIO_enableHiZ(IOMUX_PINCM37);
//    DL_GPIO_enableHiZ(IOMUX_PINCM38);

    System::i2c1.partialInitController();
    System::i2c1.setSCLTarget(10e3);
    DL_I2C_enableController(System::i2c1.reg);

    uint8_t buff[8];
    char str[MAX_STR_LEN_COMMON];

    while(true){
        vTaskDelay(pdMS_TO_TICKS(500));

        uint8_t addr = 0x10;

        uint8_t success1 = System::i2c1.tx_ctrl_blocking(0x10, buff, 1);
        delay_cycles(80 * 60);
        uint8_t success2 = System::i2c1.rx_ctrl_blocking(0x11, buff, 2);

        snprintf(str, sizeof(str), "rx (success:%1d:%1d) : %d" NEWLINE, success1, success2, ((uint16_t *)buff)[0]);
//        snprintf(str, sizeof(str), "rx : %d" NEWLINE, ((uint16_t *)buff)[0]);
        System::uart_ui.nputs(str, sizeof(str));

    }

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

