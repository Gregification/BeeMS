/*
 * ethernet_w5500_test.cpp
 *
 *  Created on: Aug 31, 2025
 *      Author: turtl
 */

#include "ethernet_w5500_test.hpp"

#include <cstdio>

#include <Core/system.hpp>
#include "Middleware/W5500/socket.h"

auto &spi   = System::spi0;
auto &cs    = System::GPIO::PA4;

void Task::ethernetw5500_test(void *){
    System::uart_ui.nputs(ARRANDN("ethernetw5500_test start" NEWLINE));

    spi.setSCLKTarget(1e6);

    // setup CS pin
    DL_GPIO_initDigitalOutputFeatures(
            cs.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_clearPins(GPIOPINPUX(cs));
    DL_GPIO_enableOutput(GPIOPINPUX(cs));

    cs.clear();
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t tx[4];
    uint8_t rx[] = {1,1,1,1};

    {
                    char str[10];
                    snprintf(ARRANDN(str),"%d" NEWLINE, (uint32_t)uxTaskGetStackHighWaterMark(NULL));
                    System::uart_ui.nputs(ARRANDN(str));
                }
    spi.transfer(NULL, rx, sizeof(rx), pdMS_TO_TICKS(100));
    for(int i = 0; i < sizeof(rx); i++){
        char str[5];
        snprintf(ARRANDN(str), "%d", rx[i]);
        System::uart_ui.nputs(ARRANDN(str));
        System::uart_ui.nputs(ARRANDN(NEWLINE));
    }

//    wizchip_init(100, 0);
//    uint8_t sn = 0;
//    if(sn != socket(sn, Sn_MR_UDP, 4206, SF_BROAD_BLOCK)){
//        while(1);
//    }
//    uint8_t ip[] = {192,168,1,6};
//    sendto(sn, ARRANDN((uint8_t *)("meow")), ip, 6024);


    System::uart_ui.nputs(ARRANDN("ethernetw5500_test end" NEWLINE));
    vTaskDelete(NULL);
}

