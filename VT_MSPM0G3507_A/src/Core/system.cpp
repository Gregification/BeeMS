/*
 * system.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

#include "system.hpp"

#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>

/*--- variables ------------------------------------------------------------------------*/

namespace System {
    namespace CLK {
        extern uint32_t LFCLK   = 32768;
        extern uint32_t ULPCLK  = 32e6;
        extern uint32_t &MCLK   = CPUCLK;
        extern uint32_t CPUCLK  = configCPU_CLOCK_HZ;
        extern uint32_t CANCLK  = 0;
        extern uint32_t MFPCLK  = 4e6;
    }

    #ifdef PROJECT_ENABLE_UART0
        UART::UART uart0 = {.reg = UART0};
    #endif

    UART::UART &uart_ui = uart0;
}


/*--- functions ------------------------------------------------------------------------*/

void System::init() {
    /*
     * BOR typical trigger level (v) (DS.7.6.1)
     *  0: 1.57
     *  1: 2.14
     *  2: 2.73
     *  3: 2.92
     */
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL::DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_enableMFCLK();
    DL_SYSCTL_enableMFPCLK();
    // catch 22 forces a <=32Mhz cpu if we want 500Kb can
    // 32Mhz
    {
        DL_SYSCTL_disableSYSPLL();
//            constexpr DL_SYSCTL_SYSPLLConfig pll_config = {
//                .rDivClk2x  = 0x0,
//                .rDivClk1   = 0xF,
//                .rDivClk0   = 0x0,
////                .enableCLK2x= , // idk
//                .sysPLLMCLK = DL_SYSCTL_SYSPLL_MCLK::DL_SYSCTL_SYSPLL_MCLK_CLK0,
//                .sysPLLRef  = DL_SYSCTL_SYSPLL_REF::DL_SYSCTL_SYSPLL_REF_SYSOSC,
//                .qDiv       = 0x01,
//                .pDiv       = DL_SYSCTL_SYSPLL_PDIV::DL_SYSCTL_SYSPLL_PDIV_2,
//                .inputFreq  = DL_SYSCTL_SYSPLL_INPUT_FREQ::DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ
//            };
        DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ::DL_SYSCTL_SYSOSC_FREQ_BASE);
//            DL_SYSCTL_switchMCLKfromSYSOSCtoHSCLK(DL_SYSCTL_HSCLK_SOURCE::DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
//            DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER::DL_SYSCTL_MCLK_DIVIDER_DISABLE);
//            DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV::DL_SYSCTL_ULPCLK_DIV_1);
//            DL_SYSCTL_configSYSPLL(&pll_config);
    }
    while ((DL_SYSCTL_getClockStatus() & (DL_SYSCTL_CLK_STATUS_LFOSC_GOOD))
               != (DL_SYSCTL_CLK_STATUS_LFOSC_GOOD))
    {}

    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(POWER_STARTUP_DELAY);

    #ifdef PROJECT_ENABLE_UART0
        System::uart0.partialInit();
        DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX); // PA10
        DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX); // PA11
        DL_UART_enable(System::uart0.reg);
    #endif
}

/*--- UART ---------------------------------------------*/

void System::UART::UART::setBaudTarget(uint32_t target_baud, uint32_t clk) {
    //TODO pg1351 https://www.ti.com/lit/ug/slau846b/slau846b.pdf?ts=1749245238762&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FMSPM0G3507
    // 115200 baud
    uint32_t nume = clk;
    uint32_t deno = target_baud;

    switch(DL_UART_getOversampling(reg)){
        default:            break; // should never reach this, your cooked, maybe screwed up the initialization or something
        case DL_UART_OVERSAMPLING_RATE::DL_UART_OVERSAMPLING_RATE_3X:
            deno *= 3;      break;
        case DL_UART_OVERSAMPLING_RATE::DL_UART_OVERSAMPLING_RATE_8X:
            deno *= 8;      break;
        case DL_UART_OVERSAMPLING_RATE::DL_UART_OVERSAMPLING_RATE_16X:
            deno *= 16;     break;
    };

    int32_t integer, fractional;
    integer = nume / deno;
    fractional = ( (nume * 64) + 1 ) / deno;

    DL_UART_setBaudRateDivisor(reg, integer, fractional);
}

/**
 * usage example
 *      System::UART::partialInit(UART0);
 *      DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX); // PA10
 *      DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX); // PA11
 *
 *      setBaudTarget(UART0, 115200);
 *      DL_UART_enable(UART0);
 */
void System::UART::UART::partialInit() {
    DL_UART_disable(reg);
    DL_UART_disablePower(reg);
    DL_UART_reset(reg);
    DL_UART_enablePower(reg);

    constexpr DL_UART_ClockConfig config_uart_clk = {
            .clockSel   = DL_UART_CLOCK::DL_UART_CLOCK_MFCLK,
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
    DL_UART_setClockConfig(reg, &config_uart_clk);
    DL_UART_init(reg, &config_uart);

    DL_UART_setOversampling(reg, DL_UART_OVERSAMPLING_RATE::DL_UART_OVERSAMPLING_RATE_16X);

    DL_UART_enableFIFOs(reg); // not required but very useful
}

void System::FailHard(const char *str) {
    taskDISABLE_INTERRUPTS();
    for(;;){
        System::uart_ui.nputs(STRANDN(NEWLINE "fatal error: "));
        System::uart_ui.nputs(str, MAX_STR_ERROR_LEN);
    }
}

void System::UART::UART::nputs(const char *str, uint32_t n) {
    // do NOT make this a task, keep it simple. we'll make another function later that does it passively as a task

    for(uint32_t i = 0; (i < n) && (str[i] != '\0'); i++){
        DL_UART_transmitDataBlocking(reg, str[i]);
    }
}

void System::SPI::tx_blocking(const void *data, uint16_t size) {

}

void System::SPI::rx_blocking(void *data, uint16_t size) {

}


/*--- idiot detection ------------------------------------------------------------------*/

#if !defined(PROJECT_ENABLE_UART0)
    #error "uart0 should always be enabled and used for the UI. better be a good reason otherwise."
    /* uart0 is used by the LP */
#endif
