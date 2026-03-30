/*
 * MasterBoard.cpp
 *
 *  Created on: Dec 13, 2025
 *      Author: turtl
 */

#include "MasterBoard.hpp"
#include "task.h"
#include "Core/std alternatives/string.hpp"

// absurd naming!!! yippie!

namespace MstrB {
    OpProfile_t opProfile = {
            .GLV_IL_RELAY_allow_usr_ovrd = false,   // allow local control
            .GLV_IL_RELAY_usr_requested = true,     // ^
        };
    OpVars_t opVars = {
            .GLV_IL_RELAY_engage = false,
        };

    System::SPI::SPI & Eth::spi   = System::SPI::spi0;
    System::SPI::SPI & FS::spi    = SPI::spi1;

    namespace MCHS {
        MCP33151 ADCpercise = {
               .spi = SPI::spi1,
               .cs  = GPIO::PB19,
            };
        MCP33151 ADCimpercise = {
               .spi = SPI::spi1,
               .cs  = GPIO::PB18,
            };
    }
}

//#define TS_ADC  ADC0
//ADC12_Regs * const MstrB::TempSense::adc  = TS_ADC;

namespace System::UART {
    UART & uart_ui = uart2;
}

/*
 * Everything would probably be fine without the explicit GPIO setup, mostly for consistency
 */
void MstrB::init() {
    {
        using namespace MCHS;

        DL_GPIO_initDigitalOutputFeatures(
                    ADCpercise.cs.iomux,
                    DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE, // active low
                    DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                    DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                    DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
                );
        DL_GPIO_clearPins(GPIOPINPUX(ADCpercise.cs));
        DL_GPIO_enableOutput(GPIOPINPUX(ADCpercise.cs));

        DL_GPIO_initDigitalOutputFeatures(
                    ADCimpercise.cs.iomux,
                    DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE, // active low
                    DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                    DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                    DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
                );
        DL_GPIO_clearPins(GPIOPINPUX(ADCimpercise.cs));
        DL_GPIO_enableOutput(GPIOPINPUX(ADCimpercise.cs));

        ADCimpercise.spi.setSCLKTarget(16e6);
        ADCpercise.spi.setSCLKTarget(16e6);
    }

    {
        using namespace Indi;

        DL_GPIO_initDigitalOutputFeatures(
                LED::i1.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(LED::i1));
        DL_GPIO_enableOutput(GPIOPINPUX(LED::i1));

        DL_GPIO_initDigitalOutputFeatures(
                LED::i2.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(LED::i2));
        DL_GPIO_enableOutput(GPIOPINPUX(LED::i2));

        DL_GPIO_initDigitalOutputFeatures(
                LED::fault.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(LED::fault));
        DL_GPIO_enableOutput(GPIOPINPUX(LED::fault));

        DL_GPIO_initDigitalOutputFeatures(
                LED::scheduler.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(LED::scheduler));
        DL_GPIO_enableOutput(GPIOPINPUX(LED::scheduler));
    }

    {
        using namespace IL;

        DL_GPIO_initDigitalOutputFeatures(
                _control.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(_control));
        DL_GPIO_enableOutput(GPIOPINPUX(_control));

        DL_GPIO_initDigitalInputFeatures(
                _sense.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_UP,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
        DL_GPIO_disableOutput(GPIOPINPUX(_sense));
    }

    /*{
        using namespace TempSense;

        static_assert(System::CLK::ULPCLK == 40e6);
        const static DL_ADC12_ClockConfig clkConfig = { // to determine sample clock frequency
                DL_ADC12_CLOCK::DL_ADC12_CLOCK_ULPCLK,
                DL_ADC12_CLOCK_FREQ_RANGE::DL_ADC12_CLOCK_FREQ_RANGE_32_TO_40,
                DL_ADC12_CLOCK_DIVIDE::DL_ADC12_CLOCK_DIVIDE_4
            };
        // sets adc clock to 10MHz

        DL_ADC12_setClockConfig(adc, (DL_ADC12_ClockConfig *) &clkConfig);

        #if TS_ADC != ADC0
        #error "adc init is instance specifc, update the init to match"
        #endif
        DL_ADC12_configConversionMem(
            adc,
            ts1.mem,
            ts1.chan,
            DL_ADC12_REFERENCE_VOLTAGE_VDDA,
            DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
            DL_ADC12_AVERAGING_MODE_ENABLED,
            DL_ADC12_BURN_OUT_SOURCE_DISABLED,
            DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
            DL_ADC12_WINDOWS_COMP_MODE_DISABLED);

        adc->ULLMEM.CTL1 &= ~ADC12_CTL1_AVGD_MASK;
        adc->ULLMEM.CTL1 |= DL_ADC12_HW_AVG_NUM_ACC_4; // samples averaged

        DL_ADC12_setPowerDownMode(adc, DL_ADC12_POWER_DOWN_MODE_MANUAL);

        // assumes adc clock is 10MHz (set at start of scope)
        DL_ADC12_setSampleTime0(adc,250);

        DL_ADC12_enableConversions(adc);
    }*/

    {
        using namespace HRLV;

        DL_GPIO_initDigitalInputFeatures(
                presence_HRLV.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
        DL_GPIO_disableOutput(GPIOPINPUX(presence_HRLV));

        DL_GPIO_initDigitalInputFeatures(
                presence_IL.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
        DL_GPIO_disableOutput(GPIOPINPUX(presence_IL));
    }

    {
        using namespace Eth;
//
//        DL_GPIO_initDigitalOutputFeatures(
//                cs.iomux,
//                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
//                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
//                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
//                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
//            );
//        DL_GPIO_clearPins(GPIOPINPUX(cs));
//        DL_GPIO_enableOutput(GPIOPINPUX(cs));
//
//        DL_GPIO_initDigitalOutputFeatures(
//                reset.iomux,
//                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
//                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
//                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
//                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
//            );
//        DL_GPIO_clearPins(GPIOPINPUX(reset));
//        DL_GPIO_enableOutput(GPIOPINPUX(reset));

        DL_GPIO_disableOutput(GPIOPINPUX(irq));
        DL_GPIO_initDigitalInputFeatures(
                irq.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_UP,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
    }

    delay_cycles(System::CLK::CPUCLK); // MCHS ADC does a auto calibration ~40mS after power up, it takes ~400mS
}

uint32_t MstrB::POST(char * error_msg, uint16_t max_msg_len) {
    // TODO
    // - try initing w5500 , the function hangs if the chip is dead
    // - read from the MCHS inputs, the ADC's better respond and give some floating value

    #ifdef PROJECT_ENABLE_SPI0
    if(!System::SPI::spi0.MISO) {
        ALT::srtCpy(error_msg, max_msg_len, "SPI0 no MISO pin officially declared");
        return __LINE__;
    }
    #endif

    #ifdef PROJECT_ENABLE_SPI1
    if(!System::SPI::spi1.MISO) {
        ALT::srtCpy(error_msg, max_msg_len, "SPI1 no MISO pin officially declared");
        return __LINE__;
    }
    #endif

    { // check hard wired pins
        DL_GPIO_initDigitalInputFeatures(
                System::GPIO::PA31.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
        DL_GPIO_initDigitalInputFeatures(
                    System::GPIO::PB17.iomux,
                    DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                    DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                    DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                    DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
                );
        DL_GPIO_initDigitalInputFeatures(
                    System::GPIO::PB9.iomux,
                    DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                    DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                    DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                    DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
                );
        if(         !System::GPIO::PA31.get()
                ||  !System::GPIO::PB17.get()
                ||  System::GPIO::PB9.get()
            ) {
            ALT::srtCpy(error_msg, max_msg_len, "bad hardware signature");
            return __LINE__;
        }

    }

    IL::getStatus(); // force update

    MstrB::Indi::LED::i1.set();
    MstrB::Indi::LED::i2.set();
    MstrB::Indi::LED::fault.set();
    MstrB::Indi::LED::scheduler.set();
    delay_cycles(2 * System::CLK::CPUCLK);
    MstrB::Indi::LED::i1.clear();
    MstrB::Indi::LED::i2.clear();
    MstrB::Indi::LED::fault.clear();
    MstrB::Indi::LED::scheduler.clear();

    return 0;
}

uint8_t MstrB::getUnitBoardID() {
    // TODO: should be physically configurable on the board, just read back those settings.
    return 123;
}

void MstrB::MCHS::recalADC() {
    ADCpercise.calabrate();
    ADCimpercise.calabrate();
}

void MstrB::MCHS::zeroV() {
    opProfile.MCHS_percise_zero_mV = ADCpercise.readmV();
    opProfile.MCHS_impercise_zero_mV = ADCimpercise.readmV();
}

bool MstrB::IL::setEnable(bool v){
    opVars.GLV_IL_RELAY_engage = v;
    return getStatus() == v;
}

bool MstrB::IL::getEnable(){
    return opVars.GLV_IL_RELAY_engage;
}

bool MstrB::IL::getStatus() {
    // 1. if overridden, use overridden value
    // 2. if user wants it on, software must also want it on

    do {
        if(opProfile.GLV_IL_RELAY_allow_usr_ovrd) { // if use override
            if(opProfile.GLV_IL_RELAY_usr_requested)
                _control.set();
            else
                _control.clear();
            break;
        }

        if(!opProfile.GLV_IL_RELAY_usr_requested) { // if user wants it off
            _control.clear();
             break;
        }

        if(opVars.GLV_IL_RELAY_engage)
            _control.set();
        else
            _control.clear();

    } while(false);

    return _control.getOutput();
}

bool MstrB::IL::getInput() {
    return _sense.get();
}

bool MstrB::IL::isUserOverriding() {
    return opProfile.GLV_IL_RELAY_allow_usr_ovrd;
}

bool MCHScalibrationADC(System::SPI::SPI& spi) {
    uint8_t rx[8];
    bool ret = true;

    static_assert(1024 == 1024/sizeof(rx) * sizeof(rx), "non even division");
    for(uint16_t i = 0; ret && i < 1024/sizeof(rx); i++) {
        spi.transfer_blocking(NULL, rx, sizeof(rx), NULL);

        for(uint16_t j = 0; j < sizeof(rx); j++) {
            if(i * sizeof(rx) + j < 2) // ignore the first 14 bits . (ignores first 16b)
                continue;

            if(rx[j] != 0xFF) {
                System::UART::uart_ui.nputs(ARRANDN(" womp "));
                ret = false;
                break;
            }
        }
    }

    if(ret) {
        // wait for SDO to go high
        // the SPI MISO line will hold low for like ~400mS then go to high-z , the MCU
        //  has a PU on it.
        TickType_t startTime, now;
        startTime = now = xTaskGetTickCount();

        do {
            now = xTaskGetTickCount();

            if(spi.MISO->get())
                break;

            if((portTICK_PERIOD_MS * (now-startTime)) > 500)
                ret = false;

            startTime = now;
            vTaskDelay(0);
        } while(ret);

        TickType_t ms = portTICK_PERIOD_MS * (now-startTime);
        if(ms < 350 || ms > 450) // unexpected calibration time
            ret = false;
    }

    return ret;
}
