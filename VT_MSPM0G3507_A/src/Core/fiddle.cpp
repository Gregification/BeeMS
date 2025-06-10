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

// SPI
void fiddle_task(void *){
    // - SPI0 goes up to 32Mhz. speed restraints depending on MCU speed. check DOCs.

    DL_SPI_enablePower(SPI0);
    delay_cycles(POWER_STARTUP_DELAY);

    /*--- GPIO config ----------------*/

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM43, IOMUX_PINCM43_PF_SPI0_PICO);// MOSI , PB17
//    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM44, IOMUX_PINCM44_PF_SPI0_SCLK);// SCLK , PB18
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM45, IOMUX_PINCM45_PF_SPI0_POCI); // MISO , PB19
//    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM48, IOMUX_PINCM48_PF_SPI0_CS2_POCI2);// CS2  , PB20

    DL_GPIO_initPeripheralOutputFunctionFeatures(   // SCLK , PB18
            IOMUX_PINCM44,
            IOMUX_PINCM44_PF_SPI0_SCLK,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_initPeripheralOutputFunctionFeatures(   // CS2  , PB20
            IOMUX_PINCM48,
            IOMUX_PINCM48_PF_SPI0_CS2_POCI2,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );

    DL_SPI_setChipSelect(SPI0, DL_SPI_CHIP_SELECT::DL_SPI_CHIP_SELECT_2);

    DL_GPIO_enableOutput(GPIOB, DL_GPIO_PIN_17 | DL_GPIO_PIN_18 | DL_GPIO_PIN_19 | DL_GPIO_PIN_20);
    delay_cycles(32e6/1000 * 10);

    /*--- SPI config -----------------*/

    DL_SPI_Config config = {
            .mode           = DL_SPI_MODE::DL_SPI_MODE_CONTROLLER,
            .frameFormat    = DL_SPI_FRAME_FORMAT::DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0,
            .parity         = DL_SPI_PARITY::DL_SPI_PARITY_NONE,
            .dataSize       = DL_SPI_DATA_SIZE::DL_SPI_DATA_SIZE_8,
            .bitOrder       = DL_SPI_BIT_ORDER::DL_SPI_BIT_ORDER_LSB_FIRST,
            .chipSelectPin  = DL_SPI_CHIP_SELECT::DL_SPI_CHIP_SELECT_2,
        };
    DL_SPI_ClockConfig clk_config = {
             .clockSel      = DL_SPI_CLOCK::DL_SPI_CLOCK_MFCLK, // MFCLK is always 4Mhz on the G3507
             .divideRatio   = DL_SPI_CLOCK_DIVIDE_RATIO::DL_SPI_CLOCK_DIVIDE_RATIO_1,
        };
    DL_SPI_setClockConfig(SPI0, &clk_config);
    DL_SPI_init(SPI0, &config);
    DL_SPI_setBitRateSerialClockDivider(SPI0, 19); // CLK / 20 , 4M->100k
    DL_SPI_disablePacking(SPI0);
    DL_SPI_enable(SPI0);
    for(int a = 0; a < 3; a++){
        DL_SPI_transmitDataBlocking8(SPI0, a);
    }

    vTaskDelete(NULL);
}

