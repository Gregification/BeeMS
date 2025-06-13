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

    DL_I2C_enablePower(I2C1);
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

    DL_I2C_ClockConfig clk_config = {
             .clockSel      = DL_I2C_CLOCK::DL_I2C_CLOCK_BUSCLK, // note
             .divideRatio   = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_8
        };

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
//    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM45, IOMUX_PINCM45_PF_SPI0_POCI); // MISO , PB19
//    DL_GPIO_initPeripheralOutputFunctionFeatures(   // SCLK , PB18
//            IOMUX_PINCM44,
//            IOMUX_PINCM44_PF_SPI0_SCLK,
//            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
//            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
//            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
//            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
//        );
//    DL_GPIO_initPeripheralOutputFunctionFeatures(   // CS2  , PB20
//            IOMUX_PINCM48,
//            IOMUX_PINCM48_PF_SPI0_CS2_POCI2,
//            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
//            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
//            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
//            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
//        );
//
//    DL_SPI_setChipSelect(System::spi0.reg, DL_SPI_CHIP_SELECT::DL_SPI_CHIP_SELECT_2);
//
//    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_17 | DL_GPIO_PIN_18 | DL_GPIO_PIN_19 | DL_GPIO_PIN_20);
//    delay_cycles(32e6/1000 * 10);
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
//    DL_SPI_setBitRateSerialClockDivider(System::spi0.reg, 19); // CLK / 20 , 4M->100k
//    DL_SPI_disablePacking(System::spi0.reg);
//    DL_SPI_enable(System::spi0.reg);
//
//    uint8_t tx[] = {1,2,3,4,5};
//    System::spi0.tx_blocking(ARRANDN(tx));
//
//    vTaskDelete(NULL);
//}

