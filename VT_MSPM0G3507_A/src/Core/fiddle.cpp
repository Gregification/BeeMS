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

#include "system.hpp"

namespace System {
    OCCUPY(PINCM21) //PA10
    OCCUPY(PINCM22) //PA11
}

void fiddle_task(void *){
    DL_UART_reset(UART0);
    DL_UART_enablePower(UART0);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX); // PA10
    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX); // PA11

    constexpr DL_UART_ClockConfig config_uart_clk = {
        .clockSel   = DL_UART_CLOCK::DL_UART_CLOCK_BUSCLK,
        .divideRatio= DL_UART_CLOCK_DIVIDE_RATIO::DL_UART_CLOCK_DIVIDE_RATIO_8,
    };
    constexpr DL_UART_Config config_uart = {
        .mode        = DL_UART_MODE::DL_UART_MAIN_MODE_NORMAL,
        .direction   = DL_UART_DIRECTION::DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_FLOW_CONTROL::DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity      = DL_UART_PARITY::DL_UART_MAIN_PARITY_NONE,
        .wordLength  = DL_UART_WORD_LENGTH::DL_UART_MAIN_WORD_LENGTH_8_BITS,
        .stopBits    = DL_UART_STOP_BITS::DL_UART_MAIN_STOP_BITS_ONE
    };
    DL_UART_setClockConfig(UART0, &config_uart_clk);
    DL_UART_init(UART0, &config_uart);

    DL_UART_setOversampling(UART0, DL_UART_OVERSAMPLING_RATE::DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_setBaudRateDivisor(UART0, 2, 11); // 115200 baud

    DL_UART_enableFIFOs(UART0);
    DL_UART_setRXFIFOThreshold(UART0, DL_UART_RX_FIFO_LEVEL::DL_UART_RX_FIFO_LEVEL_1_4_FULL);
    DL_UART_setTXFIFOThreshold(UART0, DL_UART_TX_FIFO_LEVEL::DL_UART_TX_FIFO_LEVEL_ONE_ENTRY);

    DL_UART_enable(UART0);
    for(;;){
        DL_UART_transmitDataBlocking(UART0, 'N');
        DL_UART_transmitDataBlocking(UART0, '\n');
        DL_UART_transmitDataBlocking(UART0, '\r');
    }

    vTaskDelete(NULL);
}
