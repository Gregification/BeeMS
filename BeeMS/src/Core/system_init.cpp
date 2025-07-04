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
#include <driverlib/emac.h>
#include <driverlib/interrupt.h>
#include <inc/hw_ints.h>
#include <FreeRTOS.h>
#include <NetworkInterface.h>

void system_init_onchip(){
    // set up main oscillator
    //  the LP uses a 32.768 KHz radial can
    // - >10MHz required for ETH PHY
    SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

    // set system clock to the FreeRTOS settings
    System::CPU_FREQ = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
        SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
        SYSCTL_CFG_VCO_240), configCPU_CLOCK_HZ);

    // power up all ports
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    
    while(  !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOL)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOM)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP)
            || !MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ)
        ) {}

    // init uarts
    #ifdef PROJECT_ENABLE_UART0
        System::uart0.preinit();
        static_assert(System::uart0_regs.UART_CLOCK_src == UART_CLOCK_PIOSC, "unknown reference clock");
        UARTConfigSetExpClk(UART0_BASE,
                System::PIOSC_FREQ, // asserted
                System::UART::BAUD_UI,
                (UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8)
            );
    #endif

    // init ethernet
    // it'll be nice if someone could figure out how to integrate a mongoose service onto this without mangling plus-tcp.

    GPIOPinConfigure(GPIO_PF0_EN0LED0);
    GPIOPinConfigure(GPIO_PF4_EN0LED1);
    GPIOPinConfigure(GPIO_PF1_EN0LED2);
    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_1);

//    GPIOPinConfigure(GPIO_PG0_EN0PPS);
//    GPIOPinTypeEthernetMII(GPIO_PORTG_BASE, GPIO_PIN_0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EMAC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);

    EMACFrameFilterSet(EMAC0_BASE, EMAC_FRMFILTER_RX_ALL);

    if(pdFAIL == FreeRTOS_IPInit(System::ETHC::ip.raw, System::ETHC::mask.raw, System::ETHC::gateway.raw, System::ETHC::dns.raw, System::ETHC::mac.raw)) {
        System::FailHard("failed FreeRTOS IP init");
    }

}
