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

void System::init() {
    #ifdef PROJECT_ENABLE_MCAN0
        DL_GPIO_initPeripheralInputFunction(IOMUX_PINCM60, IOMUX_PINCM60_PF_CANFD0_CANRX); // CANRX, PA27
        DL_GPIO_initPeripheralOutputFunction(IOMUX_PINCM59, IOMUX_PINCM59_PF_CANFD0_CANTX); // CANTX, PA26
        // MCAN has to be enabled beofre the clocks are modified or the ram wont initialize
        DL_MCAN_enablePower(CANFD0);
        delay_cycles(POWER_STARTUP_DELAY);
    #endif

    /*
     * BOR typical trigger level (v) (DS.7.6.1)
     *  0: 1.57
     *  1: 2.14
     *  2: 2.73
     *  3: 2.92
     */
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL::DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    // clock configuration
    {
        DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ::DL_SYSCTL_SYSOSC_FREQ_BASE); // SYSOSC 32Mhz

        DL_SYSCTL_disableHFXT();
        DL_SYSCTL_disableSYSPLL();

        //target 160Mhz VCO
        constexpr DL_SYSCTL_SYSPLLConfig pll_config = {
                .rDivClk2x  = 0x0,
                .rDivClk1   = 0x0,
                .rDivClk0   = 0x0,
                .enableCLK2x= DL_SYSCTL_SYSPLL_CLK2X_DISABLE,
                .enableCLK1 = DL_SYSCTL_SYSPLL_CLK1_ENABLE,
                .enableCLK0 = DL_SYSCTL_SYSPLL_CLK0_ENABLE,
                .sysPLLMCLK = DL_SYSCTL_SYSPLL_MCLK::DL_SYSCTL_SYSPLL_MCLK_CLK0,
                .sysPLLRef  = DL_SYSCTL_SYSPLL_REF::DL_SYSCTL_SYSPLL_REF_SYSOSC,
                .qDiv       = 0x9,
                .pDiv       = DL_SYSCTL_SYSPLL_PDIV::DL_SYSCTL_SYSPLL_PDIV_2,
                .inputFreq  = DL_SYSCTL_SYSPLL_INPUT_FREQ::DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
            };
        DL_SYSCTL_configSYSPLL(&pll_config);
        DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV::DL_SYSCTL_ULPCLK_DIV_1);
        DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER::DL_SYSCTL_MCLK_DIVIDER_DISABLE);
        DL_SYSCTL_enableMFCLK();
        DL_SYSCTL_switchMCLKfromSYSOSCtoHSCLK(DL_SYSCTL_HSCLK_SOURCE::DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    }

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

    #ifdef PROJECT_ENABLE_I2C0
        // TODO: code to set this up. should look like I2C1's init code below
    #endif
    #ifdef PROJECT_ENABLE_I2C1
    {
        DL_I2C_enablePower(System::i2c1.reg);
        delay_cycles(POWER_STARTUP_DELAY);
        /* timeout calculation. TDS.20.2.3.6/1520. or see the DL comments in their api.
         * formula -> "X / (1 / <clk> * 520 * 1e6) + 1"
         *      X: max wait in uS
         */
        DL_I2C_setTimeoutACount(System::i2c1.reg, 50.0 * configCPU_CLOCK_HZ / 520.0e6 + 1);
        DL_I2C_enableTimeoutA(System::i2c1.reg); // SCL low timeout detection

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

        NVIC_EnableIRQ(I2C1_INT_IRQn);
        DL_I2C_enableInterrupt(System::i2c1.reg,
                  DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER
                | DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER
                | DL_I2C_INTERRUPT_CONTROLLER_TX_DONE
                | DL_I2C_INTERRUPT_CONTROLLER_RX_DONE
                | DL_I2C_INTERRUPT_TIMEOUT_A
                | DL_I2C_INTERRUPT_TIMEOUT_B
            );
    }
    #endif

    #ifdef PROJECT_ENABLE_SPI0
    {
        // high speed SPI

        DL_SPI_enablePower(spi0.reg);
        delay_cycles(POWER_STARTUP_DELAY);

        /*--- GPIO config ----------------*/

        DL_GPIO_initPeripheralOutputFunctionFeatures(//    SCLK , PA12
                IOMUX_PINCM34,
                IOMUX_PINCM34_PF_SPI0_SCLK,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_initPeripheralOutputFunctionFeatures(//    MOSI, PA14
                IOMUX_PINCM36,
                IOMUX_PINCM36_PF_SPI0_PICO,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_initPeripheralInputFunctionFeatures(//      MISO , PA13
                IOMUX_PINCM35,
                IOMUX_PINCM35_PF_SPI0_POCI,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_DISABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
        DL_GPIO_enableHiZ(IOMUX_PINCM35);
        DL_GPIO_enableOutput(GPIOPINPUX(GPIO::PA12));
        DL_GPIO_enableOutput(GPIOPINPUX(GPIO::PA13));
        DL_GPIO_enableOutput(GPIOPINPUX(GPIO::PA14));

        /*--- SPI config -----------------*/

        DL_SPI_ClockConfig clk_config = {
                 .clockSel      = DL_SPI_CLOCK::DL_SPI_CLOCK_BUSCLK, // 40e6
                 .divideRatio   = DL_SPI_CLOCK_DIVIDE_RATIO::DL_SPI_CLOCK_DIVIDE_RATIO_1,
            };
        DL_SPI_setClockConfig(spi0.reg, &clk_config);
        DL_SPI_Config config = {
                .mode           = DL_SPI_MODE::DL_SPI_MODE_CONTROLLER,
                .frameFormat    = DL_SPI_FRAME_FORMAT::DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0,
                .parity         = DL_SPI_PARITY::DL_SPI_PARITY_NONE,
                .dataSize       = DL_SPI_DATA_SIZE::DL_SPI_DATA_SIZE_8,
                .bitOrder       = DL_SPI_BIT_ORDER::DL_SPI_BIT_ORDER_LSB_FIRST,
                .chipSelectPin  = DL_SPI_CHIP_SELECT::DL_SPI_CHIP_SELECT_NONE,
            };
        DL_SPI_init(spi0.reg, &config);
        DL_SPI_disablePacking(spi0.reg);
        spi0.setSCLKTarget(125e3);
        DL_SPI_enable(spi0.reg);

//        NVIC_EnableIRQ(SPI0_INT_IRQn);
        NVIC_DisableIRQ(SPI0_INT_IRQn);
        DL_SPI_enableInterrupt(System::spi0.reg,
                  DL_SPI_INTERRUPT_RX
                | DL_SPI_INTERRUPT_TX
            );
    }
    #endif

    #ifdef PROJECT_ENABLE_MCAN0
    {
        DL_MCAN_enableModuleClock(CANFD0);

        {
            static_assert(System::CLK::CANCLK == 80e6, "CAN peripheral clock misconfigured"); // adjust the dividers so that the clock is either 20MHz,40MHz, or 80MHz
            static DL_MCAN_ClockConfig clkConfig = {
               .clockSel= DL_MCAN_FCLK::DL_MCAN_FCLK_SYSPLLCLK1,
               .divider = DL_MCAN_FCLK_DIV::DL_MCAN_FCLK_DIV_1,
            };
            DL_MCAN_setClockConfig(CANFD0, &clkConfig);

        }

        // have no idea if this is required for the setup
        DL_MCAN_RevisionId revId;
        /* Get MCANSS Revision ID. */
        DL_MCAN_getRevisionId(CANFD0, &revId);

        /* Wait for Memory initialization to be completed. */
        while(false == DL_MCAN_isMemInitDone(CANFD0))
            ;

        /* Put MCAN in SW initialization mode. */
        DL_MCAN_setOpMode(CANFD0, DL_MCAN_OPERATION_MODE_SW_INIT);
        while(DL_MCAN_OPERATION_MODE_SW_INIT != DL_MCAN_getOpMode(CANFD0))
            ;

        { /* Initialize MCAN module. */
            constexpr DL_MCAN_InitParams initparam = {
                  /* Initialize MCAN Init parameters.    */
                  .fdMode            = true,    // CAN FD mode?
                  .brsEnable         = true,    // enable bit rate switching?
                  .txpEnable         = true,    // pause for 2 bit times after successful transmission?
                  .efbi              = false,   // edge filtering? 2 consecutive Tq to accept sync
                  .pxhddisable       = false,   // do not Tx error frame on protocol error?
                  .darEnable         = true,    // auto retransmission of failed frames?
                  .wkupReqEnable     = false,   // enable wake up request?
                  .autoWkupEnable    = false,   // enable auto-wakeup?
                  .emulationEnable   = false,
                  .wdcPreload        = 255,     // start value of message ram WDT

                  /* Transmitter Delay Compensation parameters. tm.26.4.6/1439 */
                  .tdcEnable         = false,   // transmitter delay compensation, oscilloscope the time diff between edges of TX->RX signals
                  .tdcConfig.tdcf    = 10,      // filter length
                  .tdcConfig.tdco    = 6,       // filter offset
                };
            DL_MCAN_init(CANFD0, &initparam);
        }

        { /* Configure MCAN module. */
            constexpr DL_MCAN_ConfigParams config = {
                    .monEnable         = false,     // bus monitoring mode
                    .asmEnable         = false,     // restricted operation mode
                    .tsPrescalar       = 15,        // timestamp counter prescalar
                    .tsSelect          = 0,         // timestamp source selection
                    .timeoutSelect     = DL_MCAN_TIMEOUT_SELECT_CONT, // timeout source select
                    .timeoutPreload    = 65535,     // load value of timeout counter
                    .timeoutCntEnable  = false,     // timeout counter enable
                    .filterConfig.rrfe = 0,      // reject remote frames extended
                    .filterConfig.rrfs = 0,      // reject remote frames standard
                    .filterConfig.anfe = 0,         // accept non-matching frames extended
                    .filterConfig.anfs = 0,         // accept non-matching frames standard
                };
            DL_MCAN_config(CANFD0, &config);
        }

        { /* Configure Bit timings. */
            constexpr DL_MCAN_BitTimingParams   bitTimeingParams = {
                    .nomRatePrescalar   = 4,    /* Arbitration Baud Rate Pre-scaler. */
                    .nomTimeSeg1        = 26,   /* Arbitration Time segment before sample point. */
                    .nomTimeSeg2        = 3,    /* Arbitration Time segment after sample point. */
                    .nomSynchJumpWidth  = 3,    /* Arbitration (Re)Synchronization Jump Width Range. */
                    .dataRatePrescalar  = 4,    /* Data Baud Rate Pre-scaler. */
                    .dataTimeSeg1       = 26,    /* Data Time segment before sample point. */
                    .dataTimeSeg2       = 3,    /* Data Time segment after sample point. */
                    .dataSynchJumpWidth = 3,    /* Data (Re)Synchronization Jump Width.   */
                };
            DL_MCAN_setBitTime(CANFD0, &bitTimeingParams);
        }

        { /* Configure Message RAM Sections */
            constexpr DL_MCAN_MsgRAMConfigParams  ramConfig = {
                    .flssa                = 0,  /* Standard ID Filter List Start Address. */
                    .lss                  = 2,  /* List Size: Standard ID. */
                    .flesa                = 0,  /* Extended ID Filter List Start Address. */
                    .lse                  = 0,  /* List Size: Extended ID. */
                    .txStartAddr          = 148, /* Tx Buffers Start Address. */
                    .txBufNum             = 2,  /* Number of Dedicated Transmit Buffers. */
                    .txFIFOSize           = 19, /* Tx Buffer Element Size. */
                    .txBufMode            = 0,
                    .txBufElemSize        = DL_MCAN_ELEM_SIZE_64BYTES,
                    .txEventFIFOStartAddr = 164,/* Tx Event FIFO Start Address. */
                    .txEventFIFOSize      = 2,  /* Event FIFO Size. */
                    .txEventFIFOWaterMark = 0,  /* Level for Tx Event FIFO watermark interrupt. */
                    .rxFIFO0startAddr     = 172,  /* Rx FIFO0 Start Address. */
                    .rxFIFO0size          = 3,  /* Number of Rx FIFO elements. */
                    .rxFIFO0waterMark     = 0,  /* Rx FIFO0 Watermark. */
                    .rxFIFO0OpMode        = 0,
                    .rxFIFO1startAddr     = 192,  /* Rx FIFO1 Start Address. */
                    .rxFIFO1size          = 2,  /* Number of Rx FIFO elements. */
                    .rxFIFO1waterMark     = 3,  /* Level for Rx FIFO 1 watermark interrupt. */
                    .rxFIFO1OpMode        = 0,  /* FIFO blocking mode. */
                    .rxBufStartAddr       = 208,  /* Rx Buffer Start Address. */
                    .rxBufElemSize        = DL_MCAN_ELEM_SIZE_8BYTES,  /* Rx Buffer Element Size. */
                    .rxFIFO0ElemSize      = DL_MCAN_ELEM_SIZE_8BYTES,  /* Rx FIFO0 Element Size. */
                    .rxFIFO1ElemSize      = DL_MCAN_ELEM_SIZE_8BYTES,  /* Rx FIFO1 Element Size. */
                };
            DL_MCAN_msgRAMConfig(CANFD0, &ramConfig);
        }

//        {
//            constexpr DL_MCAN_StdMsgIDFilterElement filtere = {
//                   .sfid2   = 3 << 18,
//                   .sfid1   = 3,
//                   .sfec    = 0b001,
//                   .sft     = 0b01,
//                };
//            DL_MCAN_addStdMsgIDFilter(CANFD0, 0, &filtere);
//        }

        /* Set Extended ID Mask. */
        // is ANDed with 29b message id of frame
        DL_MCAN_setExtIDAndMask(CANFD0, 0x1FFFFFFF);

        /* Take MCAN out of the SW initialization mode */
        DL_MCAN_setOpMode(CANFD0, DL_MCAN_OPERATION_MODE_NORMAL);
        while(DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(CANFD0))
            ;
    }
    #endif

}

inline bool System::Lockable::takeResource(TickType_t timeout) {
    return pdTRUE == xSemaphoreTakeRecursive(semph, timeout);
}

inline void System::Lockable::releaseResource() {
    xSemaphoreGiveRecursive(semph);
}

void System::UART::UART::setBaudTarget(uint32_t target_baud, uint32_t clk) {
    // i remember seeing there was some function in DL that did exactly the same thing.
    // I cant find it anymore :(

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

    return;
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

    System::uart_ui.nputs(ARRANDN(NEWLINE "fatal error: "));
    System::uart_ui.nputs(str, MAX_STR_ERROR_LEN);

    delay_cycles(System::CLK::CPUCLK);

    DL_SYSCTL_resetDevice(DL_SYSCTL_RESET_CAUSE_POR_SW_TRIGGERED);
}

void System::UART::UART::nputs(const char *str, uint32_t n) {
    for(uint32_t i = 0; (i < n) && (str[i] != '\0'); i++){
        DL_UART_transmitDataBlocking(reg, str[i]);
    }
}

void System::UART::UART::ngets(char *str, uint32_t n) {
    for(uint32_t i = 0; i < n; i++){
        char c = DL_UART_receiveDataBlocking(reg);

        str[i] = c;

        switch(c){
            case '\0':
            case '\n':
            case '\r':
                if(i == n)
                    str[i] ='\0';
                else
                    str[i+1] ='\0';
                return;

            case '\b':
                i -= 2;
        }

        DL_UART_transmitDataBlocking(reg, c);
    }
}

void System::UART::UART::printu32d(uint32_t v) {
    if(v == 0){
        DL_UART_transmitDataBlocking(reg, '0');
        return;
    }

    char str[10];
    int i;
    for(i = 0; v > 0; i++){
        str[i] = '0' + (v % 10);
        v /= 10;
    }
    i--;
    for(; i >= 0; i--){
        DL_UART_transmitDataBlocking(reg, str[i]);
    }
}

void System::UART::UART::printu32h(uint32_t v) {
    int started = 0;

    int i;
    DL_UART_transmitDataBlocking(reg, '0');
    DL_UART_transmitDataBlocking(reg, 'x');
    for (i = 28; i >= 0; i -= 4) {
        uint8_t B = (v >> i) & 0xF;

        if (B != 0 || started || i == 0) {
            started = 1;
            if (B < 10)
                DL_UART_transmitDataBlocking(reg, '0' + B);
            else
                DL_UART_transmitDataBlocking(reg, 'A' + (B - 10));
        }
    }
}

void System::SPI::SPI::setSCLKTarget(uint32_t target, uint32_t clk){
    uint32_t t = clk / target;
    if(clk - t * target)
        t++;
    DL_SPI_setBitRateSerialClockDivider(reg, t+1); // eh
}

void System::SPI::SPI::_irq() {
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    switch(DL_SPI_getPendingInterrupt(reg)){
        case DL_SPI_IIDX::DL_SPI_IIDX_RX:
            while(!DL_SPI_isRXFIFOEmpty(reg)){
                if(_trxBuffer.rx_i < _trxBuffer.len){
                    _trxBuffer.rx[_trxBuffer.rx_i++] = DL_SPI_receiveData8(reg);
                } else {
                    // ignore and flush
                    DL_SPI_receiveData8(reg);
                }
            }

            break;

            // transmit more stuff
        case DL_SPI_IIDX::DL_SPI_IIDX_TX:
            if(_trxBuffer.tx_i < _trxBuffer.len){ // anything left to TX?
                if(_trxBuffer.tx) { // TX array contents
                    _trxBuffer.tx_i +=  DL_SPI_fillTXFIFO8(
                            reg,
                            (uint8_t *)_trxBuffer.tx,
                            _trxBuffer.len - _trxBuffer.tx_i
                        );
                } else { // TX bogus data
                    for(; (_trxBuffer.tx_i < _trxBuffer.len) && !DL_SPI_isTXFIFOFull(reg); _trxBuffer.tx_i++){
                        DL_SPI_transmitData8(reg, TRANSFER_FILLER_BYTE);
                    }
                }
            }

            break;

        default:
            break;

    };

    // end condition
    if((_trxBuffer.rx_i >= _trxBuffer.len) && (_trxBuffer.tx_i >= _trxBuffer.len))
        xTaskNotifyIndexedFromISR(_trxBuffer.host_task, TASK_NOTIFICATION_ARRAY_INDEX_FOR_SYSTEM_SPI_IRQ, 0, eNotifyAction::eIncrement, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

bool System::SPI::SPI::transfer(void * tx, void * rx, uint16_t len, TickType_t timeout){
    TickType_t stopTime = xTaskGetTickCount() + timeout;

    while(DL_SPI_isBusy(reg)){
        vTaskDelay(0);
        if(xTaskGetTickCount() > stopTime)
            return false;
    }

    if(!takeResource(timeout))
        return false;

    _trxBuffer.host_task = xTaskGetCurrentTaskHandle();
    _trxBuffer.tx = (uint8_t *) tx;
    _trxBuffer.rx = (uint8_t *) rx;
    _trxBuffer.rx_i = 0;
    _trxBuffer.len = len;

    NVIC_EnableIRQ(SPI0_INT_IRQn);

    // sanitization
    if(!_trxBuffer.rx)
        _trxBuffer.rx_i = _trxBuffer.len;
    // tx is checked by the IRQ so its not needed here

    // start IRQ handling by transmitting something
    if(_trxBuffer.tx) {
        _trxBuffer.tx_i = DL_SPI_fillTXFIFO8(reg, _trxBuffer.tx, 1);
    } else {
        _trxBuffer.tx_i = DL_SPI_fillTXFIFO8(reg, &TRANSFER_FILLER_BYTE, 1);
    }

    if(0 == ulTaskNotifyTakeIndexed(TASK_NOTIFICATION_ARRAY_INDEX_FOR_SYSTEM_I2C_IRQ, pdTRUE, timeout)){
        NVIC_DisableIRQ(SPI0_INT_IRQn);
        // timed out
        _trxBuffer.len = 0;
        releaseResource();
        return false;
    }

    NVIC_DisableIRQ(SPI0_INT_IRQn);
    _trxBuffer.len = 0;
    releaseResource();
    return true;
}


void System::I2C::I2C::partialInitController(){
    DL_I2C_ClockConfig clk_config = {
             .clockSel      = DL_I2C_CLOCK::DL_I2C_CLOCK_BUSCLK,
             .divideRatio   = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_2
        };
    DL_I2C_setClockConfig(reg, &clk_config);
    DL_I2C_enableAnalogGlitchFilter(reg);
    DL_I2C_setControllerAddressingMode(reg, DL_I2C_CONTROLLER_ADDRESSING_MODE::DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);

    // as controller
    DL_I2C_resetControllerTransfer(reg);

    DL_I2C_setControllerTXFIFOThreshold(reg, DL_I2C_TX_FIFO_LEVEL::DL_I2C_TX_FIFO_LEVEL_BYTES_1);
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

void System::I2C::I2C::_irq() {
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

    switch(DL_I2C_getPendingInterrupt(reg)){
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            while(!DL_I2C_isControllerRXFIFOEmpty(reg)){
                if(_trxBuffer.nxt_index < _trxBuffer.data_length){
                    _trxBuffer.data[_trxBuffer.nxt_index++] = DL_I2C_receiveControllerData(reg);
                } else {
                    // ignore and flush
                    DL_I2C_receiveControllerData(reg);
                }
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            // fill TX fifo
            if(_trxBuffer.nxt_index < _trxBuffer.data_length){
                _trxBuffer.nxt_index +=  DL_I2C_fillControllerTXFIFO(
                        reg,
                        (uint8_t *)_trxBuffer.data,
                        _trxBuffer.data_length - _trxBuffer.nxt_index
                    );
            }
            break;

        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            _trxBuffer.nxt_index = 0;
        case DL_I2C_IIDX_TIMEOUT_A:
        case DL_I2C_IIDX_TIMEOUT_B:
        case DL_I2C_IIDX_CONTROLLER_NACK:
            _trxBuffer.data_length = 0;
            if(_trxBuffer.host_task)
                xTaskNotifyIndexedFromISR(_trxBuffer.host_task, TASK_NOTIFICATION_ARRAY_INDEX_FOR_SYSTEM_I2C_IRQ, 0, eNotifyAction::eIncrement, &xHigherPriorityTaskWoken);
            break;

        default:
            break;

    };

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

bool System::I2C::I2C::tx_blocking(uint8_t addr, void * data, uint8_t size, TickType_t timeout) {
    TickType_t stopTime = xTaskGetTickCount() + timeout;
    if(!takeResource(timeout))
        return false;

    _trxBuffer.host_task = xTaskGetCurrentTaskHandle();
    _trxBuffer.data      = (uint8_t *)data;
    _trxBuffer.data_length = size;
    _trxBuffer.nxt_index = 0;

    DL_I2C_flushControllerTXFIFO(reg);

    size -= DL_I2C_fillControllerTXFIFO(reg, _trxBuffer.data, size);

    while (!(DL_I2C_getControllerStatus(reg) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        vTaskDelay(0);
        if(xTaskGetTickCount() > stopTime){
            DL_I2C_flushControllerTXFIFO(reg);
            releaseResource();
            return false;
        }
    }

    DL_I2C_startControllerTransfer(
            reg,
            addr,
            DL_I2C_CONTROLLER_DIRECTION::DL_I2C_CONTROLLER_DIRECTION_TX,
            _trxBuffer.data_length
        );

    if(0 == ulTaskNotifyTakeIndexed(TASK_NOTIFICATION_ARRAY_INDEX_FOR_SYSTEM_I2C_IRQ, pdTRUE, stopTime - xTaskGetTickCount())){
        // timed out
        DL_I2C_flushControllerTXFIFO(reg);
        _trxBuffer.host_task = NULL;
        _trxBuffer.data_length = 0;
        releaseResource();

        return false;
    }

    DL_I2C_flushControllerTXFIFO(reg);
    _trxBuffer.host_task = NULL;
    _trxBuffer.data_length = 0;
    releaseResource();

    return true;
}

bool System::I2C::I2C::rx_blocking(uint8_t addr, void * data, uint8_t size, TickType_t timeout) {
    TickType_t stopTime = xTaskGetTickCount() + timeout;
    if(!takeResource(timeout))
        return false;

    bool ret = false;
    do{
        _trxBuffer.host_task = xTaskGetCurrentTaskHandle();
        _trxBuffer.data      = (uint8_t *)data;
        _trxBuffer.data_length = size;
        _trxBuffer.nxt_index = 0;

        while (!(DL_I2C_getControllerStatus(reg) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
            vTaskDelay(0);
            if(xTaskGetTickCount() > stopTime){
                break;
            }
        }

        DL_I2C_flushControllerTXFIFO(reg);

        DL_I2C_startControllerTransfer(
                reg,
                addr,
                DL_I2C_CONTROLLER_DIRECTION::DL_I2C_CONTROLLER_DIRECTION_RX,
                _trxBuffer.data_length
            );

        if(0 == ulTaskNotifyTakeIndexed(TASK_NOTIFICATION_ARRAY_INDEX_FOR_SYSTEM_I2C_IRQ, pdTRUE, stopTime - xTaskGetTickCount())){
            break;
        }

        if(_trxBuffer.nxt_index != _trxBuffer.data_length)
            break;

        ret = true;
    }while(false);

    // clean up
    DL_I2C_flushControllerTXFIFO(reg);
    _trxBuffer.host_task = NULL;
    _trxBuffer.data_length = 0;
    releaseResource();

    return ret;
}


/*--- Peripheral IRQ assignment --------------------------------------------------------*/
/* most peripherals don't need a IRQ
 */

#ifdef PROJECT_ENABLE_I2C0
    extern "C" void I2C0_IRQHandler(void){ System::i2c0._irq(); }
#endif
#ifdef PROJECT_ENABLE_I2C1
    extern "C" void I2C1_IRQHandler(void){ System::i2c1._irq(); }
#endif
#ifdef PROJECT_ENABLE_SPI0
    extern "C" void SPI0_IRQHandler(void){ System::spi0._irq(); }
#endif
#ifdef PROJECT_ENABLE_SPI1
    extern "C" void SPI1_IRQHandler(void){ System::spi1._irq(); }
#endif


    // for ease of debugging. delete if needed
    /*
    extern "C" void NMI_Handler(void)
    { while(1){} }
    extern "C" void HardFault_Handler(void) // if this is giving u a problem check if your using IRQ safe funcitons in your IRQ
    { while(1){} }
    extern "C" void GROUP0_IRQHandler(void)
    { while(1){} }
    extern "C" void GROUP1_IRQHandler(void)
    { while(1){} }
    extern "C" void TIMG8_IRQHandler(void)
    { while(1){} }
    extern "C" void UART3_IRQHandler(void)
    { while(1){} }
    extern "C" void ADC0_IRQHandler(void)
    { while(1){} }
    extern "C" void ADC1_IRQHandler(void)
    { while(1){} }
    extern "C" void CANFD0_IRQHandler(void)
    { while(1){} }
    extern "C" void DAC0_IRQHandler(void)
    { while(1){} }
    extern "C" void UART1_IRQHandler(void)
    { while(1){} }
    extern "C" void UART2_IRQHandler(void)
    { while(1){} }
    extern "C" void UART0_IRQHandler(void)
    { while(1){} }
    extern "C" void TIMG0_IRQHandler(void)
    { while(1){} }
    extern "C" void TIMG6_IRQHandler(void)
    { while(1){} }
    extern "C" void TIMA0_IRQHandler(void)
    { while(1){} }
    extern "C" void TIMA1_IRQHandler(void)
    { while(1){} }
    extern "C" void TIMG7_IRQHandler(void)
    { while(1){} }
    extern "C" void TIMG12_IRQHandler(void)
    { while(1){} }
    extern "C" void I2C0_IRQHandler(void)
    { while(1){} }
    extern "C" void AES_IRQHandler(void)
    { while(1){} }
    extern "C" void RTC_IRQHandler(void)
    { while(1){} }
    extern "C" void DMA_IRQHandler(void)
    { while(1){} }
    */

/*--- idiot detection ------------------------------------------------------------------*/

#if !defined(PROJECT_ENABLE_UART0)
    #error "uart0 should always be enabled and used for the UI. better be a good reason otherwise."
    /* uart0 is used by the LP */
#endif

// i fear for the day this happens
static_assert(pdTRUE == true,
        "pdTRUE != true . the FreeRTOS definition of \"true\" is not the same value as c/c++s \
        definition. code probably wont work. maybe FreeRTOS files were edited. consider reinstall."
    );
static_assert(pdFALSE == false,
        "pdFALSE != false . the FreeRTOS definition of \"false\" is not the same value as c/c++s \
        definition. code probably wont work. maybe FreeRTOS files were edited. consider reinstall."
    );

#ifdef MSPM0G3507_LQFP64
    #define MSPM0G3507_FOOTPRINT_C0 1
#else
    #define MSPM0G3507_FOOTPRINT_C0 0
#endif
#ifdef MSPM0G3507_LQFP48
    #define MSPM0G3507_FOOTPRINT_C1 1
#else
    #define MSPM0G3507_FOOTPRINT_C1 0
#endif
#ifdef MSPM0G3507_VQFN48
    #define MSPM0G3507_FOOTPRINT_C2 1
#else
    #define MSPM0G3507_FOOTPRINT_C2 0
#endif
#ifdef MSPM0G3507_VQFN32
    #define MSPM0G3507_FOOTPRINT_C3 1
#else
    #define MSPM0G3507_FOOTPRINT_C3 0
#endif
#ifdef MSPM0G3507_VSSOP28
    #define MSPM0G3507_FOOTPRINT_C4 1
#else
    #define MSPM0G3507_FOOTPRINT_C4 0
#endif
// Total number of footprints selected
#define MSPM0G3507_FOOTPRINT_TOTAL ( \
    MSPM0G3507_FOOTPRINT_C0 + \
    MSPM0G3507_FOOTPRINT_C1 + \
    MSPM0G3507_FOOTPRINT_C2 + \
    MSPM0G3507_FOOTPRINT_C3 + \
    MSPM0G3507_FOOTPRINT_C4 )
#if MSPM0G3507_FOOTPRINT_TOTAL == 0
    #error "No MSPM0G3507 variant defined. Must define exactly one."
#elif MSPM0G3507_FOOTPRINT_TOTAL > 1
    #error "Multiple MSPM0G3507 variants defined. Can only define one."
    // macros are at top of the header file
#endif
