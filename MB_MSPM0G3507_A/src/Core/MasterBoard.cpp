/*
 * MasterBoard.cpp
 *
 *  Created on: Dec 13, 2025
 *      Author: turtl
 */

#include "MasterBoard.hpp"

// absurd naming!!! yippie!
System::SPI::SPI MstrB::MHCS::spi   = System::SPI::spi1;
System::SPI::SPI MstrB::Eth::spi    = System::SPI::spi0;

#define TS_ADC  ADC0
ADC12_Regs * const MstrB::TempSense::adc  = TS_ADC;

namespace System::UART {
    UART & uart_ui = uart2;
}

/*
 * Everything would probably be fine without the explicit GPIO setup, mostly for consistency
 */
void MstrB::init() {
    {
        using namespace MHCS;

        DL_GPIO_initDigitalOutputFeatures(
                    cs_precise.iomux,
                    DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE, // active low
                    DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                    DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                    DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
                );
        DL_GPIO_clearPins(GPIOPINPUX(cs_precise));
        DL_GPIO_enableOutput(GPIOPINPUX(cs_precise));

        DL_GPIO_initDigitalOutputFeatures(
                    cs_imprecise.iomux,
                    DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE, // active low
                    DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                    DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                    DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
                );
        DL_GPIO_clearPins(GPIOPINPUX(cs_imprecise));
        DL_GPIO_enableOutput(GPIOPINPUX(cs_imprecise));
    }

    {
        using namespace Indi;

        DL_GPIO_initDigitalOutput(i1.iomux);
        DL_GPIO_clearPins(GPIOPINPUX(i1));
        DL_GPIO_enableOutput(GPIOPINPUX(i1));

        DL_GPIO_initDigitalOutput(i2.iomux);
        DL_GPIO_clearPins(GPIOPINPUX(i2));
        DL_GPIO_enableOutput(GPIOPINPUX(i2));

        DL_GPIO_initDigitalOutput(bmsFault.iomux);
        DL_GPIO_clearPins(GPIOPINPUX(bmsFault));
        DL_GPIO_enableOutput(GPIOPINPUX(bmsFault));

        DL_GPIO_initDigitalOutput(RTOSRunning.iomux);
        DL_GPIO_clearPins(GPIOPINPUX(RTOSRunning));
        DL_GPIO_enableOutput(GPIOPINPUX(RTOSRunning));
    }

    {
        using namespace IL;

        DL_GPIO_initDigitalOutputFeatures(
                control.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(control));
        DL_GPIO_enableOutput(GPIOPINPUX(control));

        DL_GPIO_initDigitalInputFeatures(
                sense.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
        DL_GPIO_disableOutput(GPIOPINPUX(sense));
    }

    {
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
    }

    {
        using namespace HRLV;

        DL_GPIO_initDigitalInputFeatures(
                HRLVSense.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
        DL_GPIO_disableOutput(GPIOPINPUX(HRLVSense));

        DL_GPIO_initDigitalInputFeatures(
                ILSense.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
        DL_GPIO_disableOutput(GPIOPINPUX(ILSense));
    }

    {
        using namespace Eth;

        DL_GPIO_initDigitalOutputFeatures(
                cs.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(cs));
        DL_GPIO_enableOutput(GPIOPINPUX(cs));

        DL_GPIO_initDigitalOutputFeatures(
                reset.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(reset));
        DL_GPIO_enableOutput(GPIOPINPUX(reset));

        DL_GPIO_initDigitalInputFeatures(
                irq.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_ENABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE
            );
        DL_GPIO_disableOutput(GPIOPINPUX(irq));
    }
}

uint8_t MstrB::getUnitBoardID() {
    // TODO: should be physically configurable on the board, just read back those settings.
    return 0;
}
