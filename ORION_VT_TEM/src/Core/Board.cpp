/*
 * MasterBoard.cpp
 *
 *  Created on: Dec 13, 2025
 *      Author: turtl
 */

#include <Core/Board.hpp>
#include <stdint.h>

// absurd naming!!! yippie!

namespace System::UART {
    UART & uart_ui = uart0;
}


namespace BOARD {

    CANFD::CANFD & can = System::CANFD::canFD0;

    namespace UI::SWITCHES {
        ADC::ChannelMap const cm = {
            .adc = System::ADC::adc0,
            .idx = DL_ADC12_MEM_IDX::DL_ADC12_MEM_IDX_0,
            .channel = DL_ADC12_INPUT_CHAN_3,
            .pin = System::GPIO::PA24,
        };
    };

    namespace Therm {
        ThermBank_t TB[3] = {
            {
                .a          = System::GPIO::PA3,
                .b          = System::GPIO::PA1,
                .c          = System::GPIO::PA0,
                .cm = {
                       .adc = System::ADC::adc1,
                       .idx = DL_ADC12_MEM_IDX::DL_ADC12_MEM_IDX_1,
                       .channel = DL_ADC12_INPUT_CHAN_2,
                       .pin = System::GPIO::PA17,
                },
            },
            {
                .a          = System::GPIO::PA11,
                .b          = System::GPIO::PA15,
                .c          = System::GPIO::PA16,
                .cm = {
                       .adc = System::ADC::adc1,
                       .idx = DL_ADC12_MEM_IDX::DL_ADC12_MEM_IDX_2,
                       .channel = DL_ADC12_INPUT_CHAN_3,
                       .pin = System::GPIO::PA18,
                },
            },
            {
                .a          = System::GPIO::PA5,
                .b          = System::GPIO::PA4,
                .c          = System::GPIO::PA10,
                .cm = {
                       .adc = System::ADC::adc0,
                       .idx = DL_ADC12_MEM_IDX::DL_ADC12_MEM_IDX_3,
                       .channel = DL_ADC12_INPUT_CHAN_12,
                       .pin = System::GPIO::PA14,
                },
            },
        };
    }
}

/*
 * Everything would probably be fine without the explicit GPIO setup, mostly for consistency
 */
void BOARD::init() {

    {
        using namespace UI;

        DL_GPIO_initDigitalOutputFeatures(
                LED::fault.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(LED::fault));
        DL_GPIO_enableOutput(GPIOPINPUX(LED::fault));

        DL_GPIO_initDigitalOutputFeatures(
                LED::scheduler.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(LED::scheduler));
        DL_GPIO_enableOutput(GPIOPINPUX(LED::scheduler));
    }

    {
        using namespace UI;

        DL_GPIO_disableOutput(GPIOPINPUX(UI::SWITCHES::cb_1));
        DL_GPIO_initDigitalInputFeatures(
                UI::SWITCHES::cb_1.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_DISABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE);

        DL_GPIO_disableOutput(GPIOPINPUX(UI::SWITCHES::cb_2));
        DL_GPIO_initDigitalInputFeatures(
                UI::SWITCHES::cb_1.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_DISABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE);

        DL_GPIO_disableOutput(GPIOPINPUX(UI::SWITCHES::uid_1));
        DL_GPIO_initDigitalInputFeatures(
                UI::SWITCHES::cb_1.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_HYSTERESIS::DL_GPIO_HYSTERESIS_DISABLE,
                DL_GPIO_WAKEUP::DL_GPIO_WAKEUP_DISABLE);

        DL_GPIO_disableOutput(GPIOPINPUX(UI::SWITCHES::cm.pin));
        DL_GPIO_initDigitalInput(UI::SWITCHES::cm.pin.iomux);
        DL_GPIO_setAnalogInternalResistor(UI::SWITCHES::cm.pin.iomux,
                  DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_UP);

        DL_ADC12_disableConversions(UI::SWITCHES::cm.adc.adc);
        DL_ADC12_configConversionMem(
                UI::SWITCHES::cm.adc.adc,
                UI::SWITCHES::cm.idx,
                UI::SWITCHES::cm.channel,
                DL_ADC12_REFERENCE_VOLTAGE_VDDA,
                DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
                DL_ADC12_AVERAGING_MODE_ENABLED,
                DL_ADC12_BURN_OUT_SOURCE_DISABLED,
                DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
                DL_ADC12_WINDOWS_COMP_MODE_DISABLED
            );

        for(unsigned int i = 0; i < Therm::THERM_N; i++){
            DL_GPIO_initDigitalInput(Therm::TB[i].cm.pin.iomux);
            DL_GPIO_setAnalogInternalResistor(Therm::TB[i].cm.pin.iomux,
                      DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_UP);
            DL_ADC12_configConversionMem(
                    Therm::TB[i].cm.adc.adc,
                    Therm::TB[i].cm.idx,
                    Therm::TB[i].cm.channel,
                    DL_ADC12_REFERENCE_VOLTAGE_VDDA,
                    DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
                    DL_ADC12_AVERAGING_MODE_ENABLED,
                    DL_ADC12_BURN_OUT_SOURCE_DISABLED,
                    DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
                    DL_ADC12_WINDOWS_COMP_MODE_DISABLED
                );
        }

        DL_ADC12_enableConversions(UI::SWITCHES::cm.adc.adc);
    }
}
