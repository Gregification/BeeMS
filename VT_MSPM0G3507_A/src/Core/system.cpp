/*
 * system.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

#include "system.hpp"

#include <ti/driverlib/driverlib.h>

namespace System {
    void init() {
        /*
         * BOR typical trigger level (v) (DS.7.6.1)
         *  0: 1.57
         *  1: 2.14
         *  2: 2.73
         *  3: 2.92
         */
        DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL::DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

        DL_SYSCTL_disableHFXT();
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

    }
}
