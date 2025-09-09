/*
 * system.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

#include "system.hpp"

#include <cstdio>

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
    namespace GPIO {
        // i got tired of fighting pin muxes

        GPIO PA0  = { .port = GPIOA, .pin = DL_GPIO_PIN_0,  .iomux = IOMUX_PINCM1  };
        GPIO PA1  = { .port = GPIOA, .pin = DL_GPIO_PIN_1,  .iomux = IOMUX_PINCM2  };
        GPIO PA2  = { .port = GPIOA, .pin = DL_GPIO_PIN_2,  .iomux = IOMUX_PINCM3  };
        GPIO PA3  = { .port = GPIOA, .pin = DL_GPIO_PIN_3,  .iomux = IOMUX_PINCM4  };
        GPIO PA4  = { .port = GPIOA, .pin = DL_GPIO_PIN_4,  .iomux = IOMUX_PINCM5  };
        GPIO PA5  = { .port = GPIOA, .pin = DL_GPIO_PIN_5,  .iomux = IOMUX_PINCM6  };
        GPIO PA6  = { .port = GPIOA, .pin = DL_GPIO_PIN_6,  .iomux = IOMUX_PINCM7  };
        GPIO PA7  = { .port = GPIOA, .pin = DL_GPIO_PIN_7,  .iomux = IOMUX_PINCM8  };
        GPIO PA8  = { .port = GPIOA, .pin = DL_GPIO_PIN_8,  .iomux = IOMUX_PINCM9  };
        GPIO PA9  = { .port = GPIOA, .pin = DL_GPIO_PIN_9,  .iomux = IOMUX_PINCM10 };
        GPIO PA10 = { .port = GPIOA, .pin = DL_GPIO_PIN_10, .iomux = IOMUX_PINCM11 };
        GPIO PA11 = { .port = GPIOA, .pin = DL_GPIO_PIN_11, .iomux = IOMUX_PINCM12 };
        GPIO PA12 = { .port = GPIOA, .pin = DL_GPIO_PIN_12, .iomux = IOMUX_PINCM13 };
        GPIO PA13 = { .port = GPIOA, .pin = DL_GPIO_PIN_13, .iomux = IOMUX_PINCM14 };
        GPIO PA14 = { .port = GPIOA, .pin = DL_GPIO_PIN_14, .iomux = IOMUX_PINCM15 };
        GPIO PA15 = { .port = GPIOA, .pin = DL_GPIO_PIN_15, .iomux = IOMUX_PINCM16 };
        GPIO PA16 = { .port = GPIOA, .pin = DL_GPIO_PIN_16, .iomux = IOMUX_PINCM17 };
        GPIO PA17 = { .port = GPIOA, .pin = DL_GPIO_PIN_17, .iomux = IOMUX_PINCM18 };
        GPIO PA18 = { .port = GPIOA, .pin = DL_GPIO_PIN_18, .iomux = IOMUX_PINCM19 };
        GPIO PA19 = { .port = GPIOA, .pin = DL_GPIO_PIN_19, .iomux = IOMUX_PINCM20 };
        GPIO PA20 = { .port = GPIOA, .pin = DL_GPIO_PIN_20, .iomux = IOMUX_PINCM21 };
        GPIO PA21 = { .port = GPIOA, .pin = DL_GPIO_PIN_21, .iomux = IOMUX_PINCM22 };
        GPIO PA22 = { .port = GPIOA, .pin = DL_GPIO_PIN_22, .iomux = IOMUX_PINCM23 };
        GPIO PA23 = { .port = GPIOA, .pin = DL_GPIO_PIN_23, .iomux = IOMUX_PINCM24 };
        GPIO PA24 = { .port = GPIOA, .pin = DL_GPIO_PIN_24, .iomux = IOMUX_PINCM25 };
        GPIO PA25 = { .port = GPIOA, .pin = DL_GPIO_PIN_25, .iomux = IOMUX_PINCM26 };
        GPIO PA26 = { .port = GPIOA, .pin = DL_GPIO_PIN_26, .iomux = IOMUX_PINCM27 };
        GPIO PA27 = { .port = GPIOA, .pin = DL_GPIO_PIN_27, .iomux = IOMUX_PINCM28 };
    }

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
    // clock
    {
        DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ::DL_SYSCTL_SYSOSC_FREQ_BASE); // SYSOSC 32Mhz
        DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER::DL_SYSCTL_MCLK_DIVIDER_DISABLE);
        DL_SYSCTL_enableMFCLK();
    }

    //BOR
    {
        // levels: 0:1V62, 1:2V23, 2:2V82, 3:2V95 . (see 7.6.1)
        // didn't notice anything about min-voltage for clock speeds
        DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL::DL_SYSCTL_BOR_THRESHOLD_LEVEL_1);
    }

    DL_GPIO_disablePower(GPIOA);
    DL_GPIO_reset(GPIOA);
    DL_GPIO_enablePower(GPIOA);
    delay_cycles(POWER_STARTUP_DELAY);

    #ifdef PROJECT_ENABLE_UART0
    {
        DL_UART_disablePower(uart0.reg);
        DL_UART_reset(uart0.reg);
        DL_UART_enablePower(uart0.reg);
        delay_cycles(POWER_STARTUP_DELAY);

        constexpr DL_UART_ClockConfig config_uart_clk = {
                .clockSel   = DL_UART_CLOCK::DL_UART_CLOCK_BUSCLK,
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
        DL_UART_setClockConfig(uart0.reg, &config_uart_clk);
        DL_UART_init(uart0.reg, &config_uart);

        DL_UART_setOversampling(uart0.reg, DL_UART_OVERSAMPLING_RATE::DL_UART_OVERSAMPLING_RATE_16X);

        DL_UART_setRXFIFOThreshold(uart0.reg, DL_UART_RX_FIFO_LEVEL::DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
        DL_UART_setTXFIFOThreshold(uart0.reg, DL_UART_TX_FIFO_LEVEL::DL_UART_TX_FIFO_LEVEL_ONE_ENTRY);
        DL_UART_enableFIFOs(uart0.reg);

        DL_GPIO_initPeripheralOutputFunctionFeatures( // PA8
                IOMUX_PINCM9,
                IOMUX_PINCM9_PF_UART0_TX,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_initPeripheralInputFunctionFeatures( // PA9
                IOMUX_PINCM10,
                IOMUX_PINCM10_PF_UART0_RX,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );

        DL_UART_enable(uart0.reg);
    }
    #endif

    #ifdef PROJECT_ENABLE_I2C0
        // TODO: code to set this up. should look like I2C1's init code below
    #endif
    #ifdef PROJECT_ENABLE_I2C1
    {
        DL_I2C_disablePower(i2c1.reg);
        DL_I2C_reset(i2c1.reg);
        DL_I2C_enablePower(i2c1.reg);
        delay_cycles(POWER_STARTUP_DELAY);

        /* timeout calculation. TDS.20.2.3.6/1520. or see the DL comments in their api.
         * formula -> "X / (1 / <clk> * 520 * 1e6) + 1"
         *      X: max wait in uS
         */
        DL_I2C_setTimeoutACount(i2c1.reg, 50.0 * System::CLK::CPUCLK / 520.0e6 + 1);
        DL_I2C_enableTimeoutA(i2c1.reg); // SCL low timeout detection

        // PA15
        DL_GPIO_initPeripheralInputFunctionFeatures(
                IOMUX_PINCM16,
                IOMUX_PINCM16_PF_I2C1_SCL,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_DISABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
        // PA16
        DL_GPIO_initPeripheralInputFunctionFeatures(
                IOMUX_PINCM17,
                IOMUX_PINCM17_PF_I2C1_SDA,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_DISABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );

        DL_I2C_ClockConfig clk_config = {
                 .clockSel      = DL_I2C_CLOCK::DL_I2C_CLOCK_BUSCLK,
                 .divideRatio   = DL_I2C_CLOCK_DIVIDE::DL_I2C_CLOCK_DIVIDE_2
            };
        DL_I2C_setClockConfig(i2c1.reg, &clk_config);
        DL_I2C_enableAnalogGlitchFilter(i2c1.reg);
        DL_I2C_setControllerAddressingMode(i2c1.reg, DL_I2C_CONTROLLER_ADDRESSING_MODE::DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);

        // as controller
        DL_I2C_resetControllerTransfer(i2c1.reg);

        DL_I2C_setControllerTXFIFOThreshold(i2c1.reg, DL_I2C_TX_FIFO_LEVEL::DL_I2C_TX_FIFO_LEVEL_BYTES_1);
        DL_I2C_setControllerRXFIFOThreshold(i2c1.reg, DL_I2C_RX_FIFO_LEVEL::DL_I2C_RX_FIFO_LEVEL_BYTES_1);
        DL_I2C_enableControllerClockStretching(i2c1.reg);

        i2c1.setSCLTarget(100e3);

        DL_I2C_enableController(i2c1.reg);

        i2c1._trxBuffer.data_length = 0;
        i2c1._trxBuffer.data = 0;
        i2c1._trxBuffer.error = I2C::I2C::ERROR::NONE;

        NVIC_EnableIRQ(I2C1_INT_IRQn);
        DL_I2C_enableInterrupt(i2c1.reg,
                  DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER
                | DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER
                | DL_I2C_INTERRUPT_CONTROLLER_TX_DONE
                | DL_I2C_INTERRUPT_CONTROLLER_RX_DONE
                | DL_I2C_INTERRUPT_TIMEOUT_A
                | DL_I2C_INTERRUPT_TIMEOUT_B
                | DL_I2C_INTERRUPT_CONTROLLER_START
                | DL_I2C_INTERRUPT_CONTROLLER_STOP
            );
    }
    #endif

    #ifdef PROJECT_ENABLE_SPI0
    {
        // high speed SPI
        DL_SPI_disablePower(spi0.reg);
        DL_SPI_reset(spi0.reg);
        DL_SPI_enablePower(spi0.reg);
        delay_cycles(POWER_STARTUP_DELAY);

        /*--- GPIO config ----------------*/

        DL_GPIO_initPeripheralOutputFunctionFeatures(//    SCLK ,  PA11
                IOMUX_PINCM12,
                IOMUX_PINCM12_PF_SPI0_SCLK,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_initPeripheralOutputFunctionFeatures(//    MOSI, PA5
                IOMUX_PINCM6,
                IOMUX_PINCM6_PF_SPI0_PICO,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_initPeripheralInputFunctionFeatures(//      MISO , PA10
                IOMUX_PINCM11,
                IOMUX_PINCM11_PF_SPI0_POCI,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
        DL_GPIO_enableHiZ(IOMUX_PINCM11); // MISO
        DL_GPIO_enableOutput(GPIOPINPUX(GPIO::PA11));
        DL_GPIO_enableOutput(GPIOPINPUX(GPIO::PA5));
        DL_GPIO_enableOutput(GPIOPINPUX(GPIO::PA10));

        /*--- SPI config -----------------*/

        constexpr DL_SPI_ClockConfig clk_config = {
                 .clockSel      = DL_SPI_CLOCK::DL_SPI_CLOCK_BUSCLK,
                 .divideRatio   = DL_SPI_CLOCK_DIVIDE_RATIO::DL_SPI_CLOCK_DIVIDE_RATIO_1,
            };
        DL_SPI_setClockConfig(spi0.reg, &clk_config);
        constexpr DL_SPI_Config config = {
                .mode           = DL_SPI_MODE::DL_SPI_MODE_CONTROLLER,
                .frameFormat    = DL_SPI_FRAME_FORMAT::DL_SPI_FRAME_FORMAT_MOTO3_POL1_PHA1,
                .parity         = DL_SPI_PARITY::DL_SPI_PARITY_NONE,
                .dataSize       = DL_SPI_DATA_SIZE::DL_SPI_DATA_SIZE_8,
                .bitOrder       = DL_SPI_BIT_ORDER::DL_SPI_BIT_ORDER_MSB_FIRST,
                .chipSelectPin  = DL_SPI_CHIP_SELECT::DL_SPI_CHIP_SELECT_NONE,
            };
        DL_SPI_init(spi0.reg, &config);
        DL_SPI_disablePacking(spi0.reg);

        DL_SPI_setFIFOThreshold(spi0.reg, DL_SPI_RX_FIFO_LEVEL::DL_SPI_RX_FIFO_LEVEL_1_2_FULL, DL_SPI_TX_FIFO_LEVEL::DL_SPI_TX_FIFO_LEVEL_1_2_EMPTY);

        DL_SPI_enable(spi0.reg);

        spi0._trxBuffer.len = 0;
        spi0._trxBuffer.rx  = 0;
        spi0._trxBuffer.tx  = 0;

        NVIC_EnableIRQ(SPI0_INT_IRQn);
        DL_SPI_enableInterrupt(System::spi0.reg,
                  DL_SPI_INTERRUPT_RX
                | DL_SPI_INTERRUPT_TX
                | DL_SPI_INTERRUPT_IDLE
            );
    }
    #endif

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

void System::FailHard(const char *str) {
    System::uart_ui.nputs(ARRANDN(NEWLINE "FAIL HARD" NEWLINE));

    // turn stuff off or what not

    for(;;){
        System::uart_ui.nputs(ARRANDN(NEWLINE "fatal error: "));
        System::uart_ui.nputs(str, MAX_STR_ERROR_LEN);
        for(uint16_t i = 0; i < 5*60; i++)
            delay_cycles(System::CLK::CPUCLK);
    }
}

void System::UART::UART::nputs(const char *str, buffsize_t n) {
    for(buffsize_t i = 0; (i < n) && (str[i] != '\0'); i++){
        DL_UART_transmitDataBlocking(reg, str[i]);
    }
}

void System::UART::UART::ngets(char *str, buffsize_t n) {
    for(buffsize_t i = 0; i < n; i++){
        char data = DL_UART_receiveDataBlocking(reg);

        str[i] = data;

        if(data == '\0' || data == '\n' || data == '\r') {
            if(i == n)
                str[i] ='\0';
            else
                str[i+1] ='\0';
            return;
        }
        else
            DL_UART_transmitDataBlocking(reg, data);
    }
}

void System::SPI::SPI::setSCLKTarget(uint32_t target, uint32_t clk){
    if(!target) target++;
    else if(target > clk) target = clk;
    uint32_t t = clk / target;
    if(clk / t <= target)
        t--;

    DL_SPI_setBitRateSerialClockDivider(reg, t);
}

void System::SPI::SPI::_irq() {
    switch(DL_SPI_getPendingInterrupt(reg)){
        case DL_SPI_IIDX::DL_SPI_IIDX_IDLE:
        case DL_SPI_IIDX::DL_SPI_IIDX_RX:
            while(!DL_SPI_isRXFIFOEmpty(reg)){
                if(_trxBuffer.rx_i < _trxBuffer.len && _trxBuffer.rx){
                        _trxBuffer.rx[_trxBuffer.rx_i++] = DL_SPI_receiveData8(reg);
                } else {
                    // ignore and flush
                    DL_SPI_receiveData8(reg);
                }
            }

            break;

        case DL_SPI_IIDX::DL_SPI_IIDX_TX:
            if(_trxBuffer.tx_i < _trxBuffer.len){
                if(_trxBuffer.tx) {
                    // TX array contents
                    _trxBuffer.tx_i +=  DL_SPI_fillTXFIFO8(
                            reg,
                            ((uint8_t *)_trxBuffer.tx) + _trxBuffer.tx_i,
                            _trxBuffer.len - _trxBuffer.tx_i
                        );
                } else {
                    // TX bogus data
                    for(; (_trxBuffer.tx_i < _trxBuffer.len) && !DL_SPI_isTXFIFOFull(reg); _trxBuffer.tx_i++){
                        DL_SPI_transmitData8(reg, TRANSFER_FILLER_BYTE);
                    }
                }
            }

            break;

        default:
            break;

    };
}

void System::SPI::SPI::transfer(void * tx, void * rx, buffsize_t len){
    while(isBusy()){}

    _trxBuffer.tx = (uint8_t *) tx;
    _trxBuffer.rx = (uint8_t *) rx;
    _trxBuffer.rx_i = 0;
    _trxBuffer.len = len;

    if(_trxBuffer.tx) {
        _trxBuffer.tx_i = DL_SPI_fillTXFIFO8(reg, _trxBuffer.tx, len);
    } else {
        _trxBuffer.tx_i = DL_SPI_fillTXFIFO8(reg, &TRANSFER_FILLER_BYTE, 1);
        // force a TX trigger incase the FIFO trigger misses this
        reg->CPU_INT.ISET |= BV(4); // TRM.23.3.12/1329
    }
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
                _trxBuffer.nxt_index += DL_I2C_fillControllerTXFIFO(
                        reg,
                        ((uint8_t *)_trxBuffer.data) + _trxBuffer.nxt_index,
                        _trxBuffer.data_length - _trxBuffer.nxt_index
                    );
            }
            break;

        case DL_I2C_IIDX_TIMEOUT_A:
        case DL_I2C_IIDX_TIMEOUT_B:
            _trxBuffer.data_length = 0;
            _trxBuffer.error = ERROR::TIMEOUT;
            break;

        case DL_I2C_IIDX_CONTROLLER_NACK:
            _trxBuffer.data_length = 0;
            _trxBuffer.error = ERROR::NACK;
            break;

        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            _trxBuffer.data_length = 0;
            _trxBuffer.error = ERROR::NONE;
            break;

        case DL_I2C_IIDX::DL_I2C_IIDX_CONTROLLER_STOP:
            switch(_trxBuffer.error){
                case ERROR::IN_USE:
                    _trxBuffer.error = ERROR::NONE;
                    break;

                default: break;
            }
            break;
        case DL_I2C_IIDX::DL_I2C_IIDX_CONTROLLER_START:
            _trxBuffer.error = ERROR::IN_USE;
            break;

        default:
            break;

    };
}

void System::I2C::I2C::tx(uint8_t addr, void * data, buffsize_t size) {
    while (isBusy())
        {}

    DL_I2C_flushControllerTXFIFO(reg);

    _trxBuffer.data         = (uint8_t *)data;
    _trxBuffer.data_length  = size;
    _trxBuffer.error        = ERROR::IN_USE;
    _trxBuffer.nxt_index    = DL_I2C_fillControllerTXFIFO(reg, _trxBuffer.data, _trxBuffer.data_length);

    DL_I2C_startControllerTransfer(
            reg,
            addr,
            DL_I2C_CONTROLLER_DIRECTION::DL_I2C_CONTROLLER_DIRECTION_TX,
            _trxBuffer.data_length
        );
}

void System::I2C::I2C::rx(uint8_t addr, void * data, buffsize_t size) {
    while(isBusy())
        {}

    DL_I2C_flushControllerTXFIFO(reg);

    _trxBuffer.data         = (uint8_t *)data;
    _trxBuffer.data_length  = size;
    _trxBuffer.nxt_index    = 0;
    _trxBuffer.error        = ERROR::IN_USE;

    DL_I2C_startControllerTransfer(
            reg,
            addr,
            DL_I2C_CONTROLLER_DIRECTION::DL_I2C_CONTROLLER_DIRECTION_RX,
            _trxBuffer.data_length
        );
}

void System::waitUS(uint32_t us) {
    delay_cycles((System::CLK::CPUCLK / 1e6 * 0.9 + 1) * us);
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


