/*
 * system_init.cpp
 *
 *  Created on: May 10, 2025
 *      Author: turtl
 */

#include "Core/system.hpp"

#include <driverlib/gpio.h>
#include <driverlib/rom_map.h>
#include <driverlib/sysctl.h>
#include <FreeRTOS.h>

void system_init_onchip(){
    // set up main oscillator
//    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

    // set system clock to the FreeRTOS settings
//    System::CPU_FREQ = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
//        SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
//        SYSCTL_CFG_VCO_240), configCPU_CLOCK_HZ);
    // the ek-tm4c123gxl board uses a 2 external oscillators 32.768KHz(Y1) on the hibernation module and 16.0MHz on the main internal clock
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);

    // power up all ports
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    
    while(  !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)
        ) {}

    // init uart

    #ifdef PROJECT_ENABLE_UART0
        System::uart0.preinit();
        static_assert(System::uart0_regs.UART_CLOCK_src == UART_CLOCK_PIOSC, "unexpected clock source");
        UARTConfigSetExpClk(System::uart0.regs.UARTn_BASE,
                System::POSIC,
                System::UART::BAUD_UI,
                (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8)
            );
    #endif
}
