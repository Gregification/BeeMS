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
//    UART & uart_ui = uart0;
}


namespace BOARD {
    CAN_MODE can_mode = CAN_MODE::A500k_D500k;
    uint8_t id = 0;

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


        {
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
            DL_ADC12_enableConversions(UI::SWITCHES::cm.adc.adc);
        }

        for(unsigned int i = 0; i < Therm::THERMB_N; i++){

            for(auto & v : (System::GPIO::GPIO[]){Therm::TB[i].a,Therm::TB[i].b,Therm::TB[i].c}) {
                DL_GPIO_initDigitalOutputFeatures(
                        v.iomux,
                        DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
                        DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN,
                        DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
                        DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
                    );
                DL_GPIO_clearPins(GPIOPINPUX(v));
                DL_GPIO_enableOutput(GPIOPINPUX(v));
            }

            DL_ADC12_disableConversions(Therm::TB[i].cm.adc.adc);

            DL_GPIO_disableOutput(GPIOPINPUX(Therm::TB[i].cm.pin));
            DL_GPIO_initDigitalInput(Therm::TB[i].cm.pin.iomux);
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

            DL_ADC12_enableConversions(Therm::TB[i].cm.adc.adc);
        }

    }
}

BOARD::CAN_MODE BOARD::UI::SWITCHES::getCANmode() {
    const GPIO::GPIO gpios[] = {cb_1, cb_2};

    CAN_MODE ret;

    for(auto pin : gpios) {
        DL_GPIO_setAnalogInternalResistor(pin.iomux,
                  DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_UP);
    }
    delay_cycles(1000);

    uint8_t id = 0;
    for(int i = 0, j = 0; i < 3 && j < 5; i++) {
        uint8_t newid = 0;
        if(cb_1.get())
            newid |= BV(0);
        if(cb_2.get())
            newid |= BV(1);

        if(newid != id) {
            id = newid;
            i = -1;
            j++;
        }

        switch(newid) {
            default:
            case CAN_MODE::A500k_D500k:
                ret = CAN_MODE::A500k_D500k;
                break;
            case CAN_MODE::A500k_D2M:
                ret = CAN_MODE::A500k_D2M;
                break;
            case CAN_MODE::A1M_D1M:
                ret = CAN_MODE::A1M_D1M;
                break;
            case CAN_MODE::A1M_D4M:
                ret = CAN_MODE::A1M_D4M;
                break;
        };

    }

    for(auto pin : gpios) {
        DL_GPIO_setAnalogInternalResistor(pin.iomux,
                  DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN);
    }

    return ret;
}

uint8_t BOARD::UI::SWITCHES::getUID() {
    const GPIO::GPIO gpios[] = {uid_1, cm.pin};
    uint8_t id = 0;

    for(auto pin : gpios) {
        DL_GPIO_setAnalogInternalResistor(pin.iomux,
                  DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_UP);
    }
    delay_cycles(1000);

    // id must be read back identically at least x times to be accepted, otherwise defaults
    {
        int j = 0;
        for(int i = 0; i < 3; i++) {
            cm.sample_blocking();
            uint16_t adc = cm.getResult();

            uint8_t newid = 0;

            constexpr float adc_max = 4096;
            struct ADCID_t {
                uint16_t val;
                uint8_t id;
            };
            constexpr float centers[7] = {0.3503,0.3792,0.4050,0.5124,0.5377,0.6089,0.6785};
            constexpr ADCID_t thresh_and_val[7] = {
                    {
                        .val = (uint16_t)(adc_max * ((centers[0] + centers[1]) / 2)),
                        .id = 0b111
                    },{
                        .val = (uint16_t)(adc_max * ((centers[1] + centers[2]) / 2)),
                        .id = 0b011
                    },{
                        .val = (uint16_t)(adc_max * ((centers[2] + centers[3]) / 2)),
                        .id = 0b101
                    },{
                        .val = (uint16_t)(adc_max * ((centers[3] + centers[4]) / 2)),
                        .id = 0b001
                    },{
                        .val = (uint16_t)(adc_max * ((centers[4] + centers[5]) / 2)),
                        .id = 0b110
                    },{
                        .val = (uint16_t)(adc_max * ((centers[5] + centers[6]) / 2)),
                        .id = 0b010
                    },{
                        .val = (uint16_t)(adc_max * (1)),
                        .id = 0b100
                    },
            };

            for(unsigned int k = 0; k < sizeof(thresh_and_val)/sizeof(thresh_and_val[0]); k++) {
                if(adc <= thresh_and_val[k].val) {
                    newid = thresh_and_val[k].id;
                    break;
                }
            }

            if(id != newid) {
                id = newid;
                i = -1;
                j++;
                if(j >= 10) {
                    id = 0;
                    break;
                }
            }
        }
    }

    id <<= 1;
    if(uid_1.get())
        id |= BV(0);

//    for(auto pin : gpios) {
//        DL_GPIO_setAnalogInternalResistor(pin.iomux,
//                  DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_DOWN);
//    }

    return id;
}

void BOARD::setCanMode(CAN_MODE newmode) {
    auto & can = System::CANFD::canFD0;
    can.takeResource(pdMS_TO_TICKS(0));

    /* Put MCAN in SW initialization mode. */
    DL_MCAN_setOpMode(can.reg, DL_MCAN_OPERATION_MODE_SW_INIT);
    while(DL_MCAN_OPERATION_MODE_SW_INIT != DL_MCAN_getOpMode(can.reg))
        ;

    constexpr DL_MCAN_BitTimingParams   a500k_d500k = {
            .nomRatePrescalar   = 4,    /* Arbitration Baud Rate Pre-scaler. */
            .nomTimeSeg1        = 26,   /* Arbitration Time segment before sample point. */
            .nomTimeSeg2        = 3,    /* Arbitration Time segment after sample point. */
            .nomSynchJumpWidth  = 3,    /* Arbitration (Re)Synchronization Jump Width Range. */
            .dataRatePrescalar  = 4,    /* Data Baud Rate Pre-scaler. */
            .dataTimeSeg1       = 26,    /* Data Time segment before sample point. */
            .dataTimeSeg2       = 3,    /* Data Time segment after sample point. */
            .dataSynchJumpWidth = 3,    /* Data (Re)Synchronization Jump Width.   */
        };

    constexpr DL_MCAN_BitTimingParams   a500k_d2M = {
            .nomRatePrescalar   = 1,    /* Arbitration Baud Rate Pre-scaler. */
            .nomTimeSeg1        = 68,   /* Arbitration Time segment before sample point. */
            .nomTimeSeg2        = 9,    /* Arbitration Time segment after sample point. */
            .nomSynchJumpWidth  = 9,    /* Arbitration (Re)Synchronization Jump Width Range. */
            .dataRatePrescalar  = 1,    /* Data Baud Rate Pre-scaler. */
            .dataTimeSeg1       = 16,    /* Data Time segment before sample point. */
            .dataTimeSeg2       = 1,    /* Data Time segment after sample point. */
            .dataSynchJumpWidth = 1,    /* Data (Re)Synchronization Jump Width.   */
        };

    constexpr DL_MCAN_BitTimingParams a1M_d1M = {
            .nomRatePrescalar   = 1,   /* 80MHz / 1 / (1 + 69 + 10) = 1MHz */
            .nomTimeSeg1        = 69,  /* 70 TQ before sample point (87.5%) */
            .nomTimeSeg2        = 9,   /* 10 TQ after sample point */
            .nomSynchJumpWidth  = 9,
            .dataRatePrescalar  = 1,
            .dataTimeSeg1       = 69,
            .dataTimeSeg2       = 9,
            .dataSynchJumpWidth = 9,
        };

    constexpr DL_MCAN_BitTimingParams a1M_d4M = {
            .nomRatePrescalar   = 1,   /* 80MHz / 1 / (1 + 69 + 10) = 1MHz */
            .nomTimeSeg1        = 69,  /* 87.5% Sample Point */
            .nomTimeSeg2        = 9,
            .nomSynchJumpWidth  = 9,
            .dataRatePrescalar  = 1,   /* 80MHz / 1 / (1 + 15 + 4) = 4MHz */
            .dataTimeSeg1       = 15,  /* 16 TQ before sample point (80%) */
            .dataTimeSeg2       = 3,   /* 4 TQ after sample point */
            .dataSynchJumpWidth = 3,
        };

    can_mode = newmode;
    switch(newmode) {
        default:
        case CAN_MODE::A500k_D500k:
            DL_MCAN_setBitTime(can.reg, &a500k_d500k);
            break;
        case CAN_MODE::A500k_D2M:
            DL_MCAN_setBitTime(can.reg, &a500k_d2M);
            break;
        case CAN_MODE::A1M_D1M:
            DL_MCAN_setBitTime(can.reg, &a1M_d1M);
            break;
        case CAN_MODE::A1M_D4M:
            DL_MCAN_setBitTime(can.reg, &a1M_d4M);
            break;
    }

    /* Take MCAN out of the SW initialization mode */
    DL_MCAN_setOpMode(can.reg, DL_MCAN_OPERATION_MODE_NORMAL);
    while(DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(can.reg))
        ;

    can.giveResource();
}

uint8_t BOARD::Therm::getThermID(uint8_t tb_i, uint8_t t_i) {
    switch(tb_i) {
        default: break;
        case 0:
            switch(t_i) {
                case 0: return 3;
                case 1: return 1;
                case 2: return 0;
                case 3: return 2;
                case 4: return 7;
                case 5: return 6;
                case 6: return 5;
                case 7: return 4;
            };
            break;
        case 1:
            switch(t_i) {
                case 0: return 13;
                case 1: return 11;
                case 2: return 10;
                case 3: return 15;
                case 4: return 12;
                case 5: return 9;
                case 6: return 14;
                case 7: return 8;
            };
            break;
        case 2:
            switch(t_i) {
                case 0: return 21;
                case 1: return 19;
                case 2: return 18;
//                case 3: return 23; // is not a cell tap, is for fan
                case 4: return 20;
                case 5: return 17;
                case 6: return 22;
                case 7: return 16;
            };
            break;
    }

    return 0xFF;
}

void BOARD::Therm::ThermBank_t::update() {
    cm.adc.takeResource(0);

    DL_GPIO_setAnalogInternalResistor(cm.pin.iomux,
          DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_UP);

    for(uint8_t i = 0; i <= 0b111; i++) {
        if(i & BV(0))   a.set();
        else            a.clear();
        vTaskDelay(pdMS_TO_TICKS(1));
        if(i & BV(1))   b.set();
        else            b.clear();
        vTaskDelay(pdMS_TO_TICKS(1));
        if(i & BV(2))   c.set();
        else            c.clear();
        vTaskDelay(pdMS_TO_TICKS(1));

        vTaskDelay(STAB_TIME);

        cm.sample_blocking();

        degcC[i] = cm.getResult();
        if(degcC[i] <= 20)
            error[i] = true;
        if(degcC[i] >= 4070)
            error[i] = true;

       // degcC[i] = 40.0e3 * ((float)degcC[i] / (4096.0 - (float)degcC[i])); // to resistance

        // theres a look up table in the DS so i just use that, https://www.mouser.com/catalog/specsheets/semitec%20usa%20corporation_smtcd00017-7.pdf
        constexpr struct {
            int32_t tempcC;
            int32_t thermR;
        } lut[] = {
               {-500    , 367700},
               {-400    , 204700},
               {-300    , 118500},
               {-200    , 71020},
               {-100    , 43670},
               {0       , 27700},
               {100     , 18070},
               {200     , 12110},
               {300     , 8301},
               {400     , 5811},
               {500     , 4147},
               {600     , 3011},
               {700     , 2224},
               {800     , 1668},
               {900     , 1267},
        };
        int j = 0;
//        if(degcC[i] > lut[j].thermR) {
//            degcC[i] = lut[j].tempcC;
//        } else {
//            for(j = 1; j < sizeof(lut)/sizeof(lut[0]); j++) {
//                if(degcC[i] > lut[j].thermR) {
//                    degcC[i] = lut[j].tempcC + (degcC[i] - lut[j].thermR) * (lut[j-1].tempcC - lut[j].tempcC) / (lut[j-1].thermR - lut[j].thermR);
//                    break;
//                }
//            }
//            if(j == sizeof(lut)/sizeof(lut[0]))
//                degcC[i] = lut[j-1].tempcC;
//        }
    }

    DL_GPIO_setAnalogInternalResistor(cm.pin.iomux,
              DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_PULL_UP);

    cm.adc.giveResource();
}
