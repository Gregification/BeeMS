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

/*--- example intis' --- for intis that are too specialized to wrap --------------------*/
/*--- SPI ---------------------------------------------*/
/* DL , note: auto CS
     DL_SPI_enablePower(SPI0);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM43, IOMUX_PINCM43_PF_SPI0_PICO);// MOSI , PB17
    DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM45, IOMUX_PINCM45_PF_SPI0_POCI); // MISO , PB19
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
    delay_cycles(32e6/1000 * 10); // optional. since the peripherals used immediately, wait for voltage to stabilize

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
 */
/* name space System, note: auto CS
     // same upper as DL version

    System::spi0.partialInit();
    DL_SPI_Config config = {
            .mode           = DL_SPI_MODE::DL_SPI_MODE_CONTROLLER,
            .frameFormat    = DL_SPI_FRAME_FORMAT::DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0,
            .parity         = DL_SPI_PARITY::DL_SPI_PARITY_NONE,
            .dataSize       = DL_SPI_DATA_SIZE::DL_SPI_DATA_SIZE_8,
            .bitOrder       = DL_SPI_BIT_ORDER::DL_SPI_BIT_ORDER_LSB_FIRST,
            .chipSelectPin  = DL_SPI_CHIP_SELECT::DL_SPI_CHIP_SELECT_2,
        };
    DL_SPI_init(System::spi0.reg, &config);
    DL_SPI_setBitRateSerialClockDivider(System::spi0.reg, 19); // CLK / 20 , 4M->100k
    DL_SPI_disablePacking(System::spi0.reg);
    DL_SPI_enable(System::spi0.reg);

    uint8_t tx[] = {1,2,3,4,5};
    System::spi0.tx_blocking(ARRANDN(tx));
 */


/*--- variables ------------------------------------------------------------------------*/

namespace System {
    namespace CLK {
        uint32_t LFCLK   = 32768;
        uint32_t ULPCLK  = 40e6;
        uint32_t &MCLK   = CPUCLK;
        uint32_t CPUCLK  = configCPU_CLOCK_HZ;
        uint32_t CANCLK  = 80e6;
        uint32_t MFPCLK  = 4e6;
    }

    // we should move to uboot one bright sunny day

    UART::UART &uart_ui = uart0;

    #ifdef PROJECT_ENABLE_UART0
        UART::UART uart0 = {.reg = UART0};
    #endif

    #ifdef PROJECT_ENABLE_SPI0
        SPI::SPI spi0 = {.reg = SPI0};
    #endif
    #ifdef PROJECT_ENABLE_SPI1
        SPI::SPI spi1 = {.reg = SPI1};
    #endif
    #ifdef PROJECT_ENABLE_I2C1
        I2C::I2C i2c1 = {.reg = I2C1};
    #endif

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

    // clock configuration
    {
        DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ::DL_SYSCTL_SYSOSC_FREQ_BASE); // SYSOSC 32Mhz

        DL_SYSCTL_disableHFXT();
        DL_SYSCTL_enableMFPCLK();
        DL_SYSCTL_enableMFCLK();
        DL_SYSCTL_disableSYSPLL();

        //target 160Mhz VCO
        constexpr DL_SYSCTL_SYSPLLConfig pll_config = {
                .rDivClk2x  = 0x3,
                .rDivClk1   = 0x0,
                .rDivClk0   = 0x0, // unused
                .enableCLK2x= DL_SYSCTL_SYSPLL_CLK2X_ENABLE,
                .enableCLK1 = DL_SYSCTL_SYSPLL_CLK1_ENABLE,
                .enableCLK0 = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
                .sysPLLMCLK = DL_SYSCTL_SYSPLL_MCLK::DL_SYSCTL_SYSPLL_MCLK_CLK2X,
                .sysPLLRef  = DL_SYSCTL_SYSPLL_REF::DL_SYSCTL_SYSPLL_REF_SYSOSC,
                .qDiv       = 0x04,
                .pDiv       = DL_SYSCTL_SYSPLL_PDIV::DL_SYSCTL_SYSPLL_PDIV_1,
                .inputFreq  = DL_SYSCTL_SYSPLL_INPUT_FREQ::DL_SYSCTL_SYSPLL_INPUT_FREQ_32_48_MHZ
            };
        DL_SYSCTL_configSYSPLL(&pll_config);
        DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER::DL_SYSCTL_MCLK_DIVIDER_DISABLE);
        DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV::DL_SYSCTL_ULPCLK_DIV_2);

    }
    while ((DL_SYSCTL_getClockStatus() & (DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD
             | DL_SYSCTL_CLK_STATUS_HSCLK_GOOD
             | DL_SYSCTL_CLK_STATUS_LFOSC_GOOD))
               != (DL_SYSCTL_CLK_STATUS_SYSPLL_GOOD
             | DL_SYSCTL_CLK_STATUS_HSCLK_GOOD
             | DL_SYSCTL_CLK_STATUS_LFOSC_GOOD))
        {}
    DL_SYSCTL_switchMCLKfromSYSOSCtoHSCLK(DL_SYSCTL_HSCLK_SOURCE::DL_SYSCTL_HSCLK_SOURCE_SYSPLL);

    // must be done after enabling PLL
    //    DL_FlashCTL_executeClearStatus(); // ERRNO thing

    DL_GPIO_disablePower(GPIOA);
    DL_GPIO_disablePower(GPIOB);
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    delay_cycles(POWER_STARTUP_DELAY);

    #ifdef PROJECT_ENABLE_UART0
        System::uart0.partialInit();
        DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM21, IOMUX_PINCM21_PF_UART0_TX); // PA10
        DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM22, IOMUX_PINCM22_PF_UART0_RX); // PA11
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
            .divideRatio= DL_UART_CLOCK_DIVIDE_RATIO::DL_UART_CLOCK_DIVIDE_RATIO_1,
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
        System::uart_ui.nputs(ARRANDN(NEWLINE "fatal error: "));
        System::uart_ui.nputs(str, MAX_STR_ERROR_LEN);
    }
}

void System::UART::UART::nputs(const char *str, uint32_t n) {
    // do NOT make this a task, keep it simple. we'll make another function later that does it passively as a task

    for(uint32_t i = 0; (i < n) && (str[i] != '\0'); i++){
        DL_UART_transmitDataBlocking(reg, str[i]);
    }
}

void System::SPI::SPI::partialInit() {
    DL_SPI_ClockConfig clk_config = {
             .clockSel      = DL_SPI_CLOCK::DL_SPI_CLOCK_BUSCLK, // 40e6
             .divideRatio   = DL_SPI_CLOCK_DIVIDE_RATIO::DL_SPI_CLOCK_DIVIDE_RATIO_1,
        };
    DL_SPI_setClockConfig(reg, &clk_config);
}

void System::SPI::SPI::setSCLKTarget(uint32_t target, uint32_t clk){
    uint32_t t = clk / target;
    if(clk - t * target)
        t++;
    DL_SPI_setBitRateSerialClockDivider(reg, t+1); // eh
}

void System::SPI::SPI::tx_blocking(const void *data, uint16_t size, GPIO::GPIO * cs) {
    if(cs){
        while(DL_SPI_isBusy(reg));
        cs->set();
    }

    for(uint16_t i = 0; i < size; i++){
        i += DL_SPI_fillTXFIFO8(reg, ((uint8_t *)data) + i, size - i);
    }

    if(cs){
        while(DL_SPI_isBusy(reg));
        cs->clear();
    }
}

void System::SPI::SPI::rx_blocking(void *data, uint16_t size, GPIO::GPIO * cs) {
    if(cs){
        while(DL_SPI_isBusy(reg));
        cs->set();
    }

    for(uint8_t c,i; c > 0 ;)
        c = DL_SPI_drainRXFIFO8(reg, &i, 1);

    for(uint16_t i = 0; i < size; i++){
        DL_SPI_transmitDataBlocking8(reg, 0);
       ((uint8_t *)data)[i] = DL_SPI_receiveData8(reg);
    }

    if(cs){
        while(DL_SPI_isBusy(reg));
        cs->clear();
    }
}


void System::I2C::I2C::partialInitController(){
    DL_I2C_ClockConfig clk_config = {
             .clockSel      = DL_I2C_CLOCK::DL_I2C_CLOCK_BUSCLK,
             .divideRatio   = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_2
        };
    DL_I2C_setClockConfig(reg, &clk_config);
    DL_I2C_enableAnalogGlitchFilter(reg);

    // as controller
    DL_I2C_resetControllerTransfer(reg);

    DL_I2C_setControllerTXFIFOThreshold(reg, DL_I2C_TX_FIFO_LEVEL::DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(reg, DL_I2C_RX_FIFO_LEVEL::DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(reg);
}

void System::I2C::I2C::setSCLTarget(uint32_t target, uint32_t clk){
    DL_I2C_ClockConfig clk_config;
    DL_I2C_getClockConfig(reg, &clk_config);

    // if clk too slow
    if(target * 20 > clk){
        clk_config.divideRatio = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_1;
        DL_I2C_setClockConfig(reg, &clk_config);
    }

    uint32_t effective_clk;
    do {
        effective_clk = clk;
        switch(clk_config.divideRatio){
            default:
            case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_1: effective_clk /= 1; break;
            case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_2: effective_clk /= 2; break;
            case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_3: effective_clk /= 3; break;
            case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_4: effective_clk /= 4; break;
            case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_5: effective_clk /= 5; break;
            case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_6: effective_clk /= 6; break;
            case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_7: effective_clk /= 7; break;
            case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_8: effective_clk /= 8; break;
        }

        // if clk too fast
        if((effective_clk / (target * 10) - 1) > BV(5) && (clk_config.divideRatio != DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_8)) {

            // increase divider and see if that works
            switch(clk_config.divideRatio){
                case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_1: clk_config.divideRatio = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_2; break;
                case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_2: clk_config.divideRatio = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_3; break;
                case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_3: clk_config.divideRatio = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_4; break;
                case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_4: clk_config.divideRatio = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_5; break;
                case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_5: clk_config.divideRatio = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_6; break;
                case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_6: clk_config.divideRatio = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_7; break;
                case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_7: clk_config.divideRatio = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_8; break;
                default:
                case DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_8: break;
            }
            DL_I2C_setClockConfig(reg, &clk_config);
        } else
            break;

    }while (true);

    DL_I2C_setTimerPeriod(reg, effective_clk / (target * 10) - 1);
}

uint8_t System::I2C::I2C::tx_ctrl_blocking(uint8_t addr, void const * data, uint8_t size){
    uint8_t const * arr = (uint8_t const *)data;

    DL_I2C_flushControllerTXFIFO(reg);

    size -= DL_I2C_fillControllerTXFIFO(reg, arr, size);

    while (!(DL_I2C_getControllerStatus(reg) & DL_I2C_CONTROLLER_STATUS_IDLE))
        {}

    DL_I2C_startControllerTransfer(
            reg,
            addr,
            DL_I2C_CONTROLLER_DIRECTION::DL_I2C_CONTROLLER_DIRECTION_TX,
            size
        );

    while(size > 0){
        size -= DL_I2C_fillControllerTXFIFO(reg, arr, size);

        if(DL_I2C_getControllerStatus(reg) & (
                DL_I2C_CONTROLLER_STATUS_IDLE
                | DL_I2C_CONTROLLER_STATUS_ERROR
            ))
            break;
    }

    while (DL_I2C_getControllerStatus(reg) &
               DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
            ;

    return size;
}

uint8_t System::I2C::I2C::rx_ctrl_blocking(uint8_t addr, void * data, uint8_t size){


    while (!(DL_I2C_getControllerStatus(reg) & DL_I2C_CONTROLLER_STATUS_IDLE))
            {}

    DL_I2C_flushControllerTXFIFO(reg);

    DL_I2C_startControllerTransfer(
                reg,
                addr,
                DL_I2C_CONTROLLER_DIRECTION::DL_I2C_CONTROLLER_DIRECTION_RX,
                size
            );
    for(uint8_t i = 0; i < size; i++){
        while(DL_I2C_isControllerRXFIFOEmpty(reg)){
            if(DL_I2C_getControllerStatus(reg) & (
                    DL_I2C_CONTROLLER_STATUS_IDLE
                    | DL_I2C_CONTROLLER_STATUS_ERROR
                ))
                break;
        }
        ((uint8_t *)data)[i] = DL_I2C_receiveControllerData(reg);
    }

    return size;
}

/*--- idiot detection ------------------------------------------------------------------*/

#if !defined(PROJECT_ENABLE_UART0)
    #error "uart0 should always be enabled and used for the UI. better be a good reason otherwise."
    /* uart0 is used by the LP */
#endif
