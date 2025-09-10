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
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <stdio.h>

#include "Core/system.hpp"
#include "Middleware/BQ76952.hpp"

//SPI
//void Task::fiddle_task(void *) {
//    // - SPI0 goes up to 32Mhz. speed restraints depending on MCU speed. check DOCs.
//
//    DL_SPI_enablePower(System::spi0.reg);
//    delay_cycles(POWER_STARTUP_DELAY);
//
//    /*--- GPIO config ----------------*/
//
//    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM43, IOMUX_PINCM43_PF_SPI0_PICO);// MOSI , PB17
//    DL_GPIO_initPeripheralInputFunctionFeatures(//    MISO , PB19
//            IOMUX_PINCM45,
//            IOMUX_PINCM45_PF_SPI0_POCI,
//            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
//            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
//            DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_DISABLE,
//            DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
//        );
//    DL_GPIO_enableHiZ(IOMUX_PINCM45);
//    DL_GPIO_initPeripheralOutputFunctionFeatures(//    SCLK , PB18
//            IOMUX_PINCM44,
//            IOMUX_PINCM44_PF_SPI0_SCLK,
//            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
//            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
//            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
//            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
//        );
//    DL_GPIO_initDigitalOutputFeatures(//    CS2  , PB20
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

// ti battery guage framework
#include <ti/battery_gauge/gauge_level2/Gauge_Type.h>
#include "ti_msp_dl_config.h"
void Task::fiddle_task(void *){
    System::uart_ui.nputs(ARRANDN("fiddle task start" NEWLINE));

    SYSCFG_DL_MCAN0_init();

    DL_MCAN_TxBufElement txMsg = {0};

    txMsg.id = (0x123 << 18);
    txMsg.rtr = 0;
    txMsg.xtd = 0;
    txMsg.dlc = 0;
    txMsg.data[0] = 0x11;
    txMsg.data[1] = 0x22;
    txMsg.data[2] = 0x33;
    txMsg.data[3] = 0x44;
    txMsg.data[4] = 0x55;
    txMsg.data[5] = 0x66;
    txMsg.data[6] = 0x77;
    txMsg.data[7] = 0x88;

    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, 0U, &txMsg);
    DL_MCAN_TXBufAddReq(MCAN0_INST, 0U);

    System::uart_ui.nputs(ARRANDN("CAN frame queued" NEWLINE));
    System::uart_ui.nputs(ARRANDN("fiddle task end" NEWLINE));
    vTaskDelete(NULL);
}
