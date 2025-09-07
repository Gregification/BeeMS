/*
 * main.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 *      https://cataas.com/cat/says/accumulating
 */
// RTLinux when? idk how to work with that :(


#include <stdio.h>
#include <stdint.h>

#include "system.hpp"
#include "Tasks/BQ769x2_PROTOCOL_Test_V.hpp"


void thing( void * ){
    auto &led = System::GPIO::PA27;
    DL_GPIO_initDigitalOutputFeatures(
            led.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_clearPins(GPIOPINPUX(led));
    DL_GPIO_enableOutput(GPIOPINPUX(led));

    for(;;){
        for(int i = 0; i < 1e3;i++){
            led.set();
            delay_cycles(10);
            led.clear();
            delay_cycles(100);
        }
        delay_cycles(16e6);
    }

}



/*************************************************************/

int main(){
    System::init();

    System::uart_ui.setBaudTarget(115200);
    System::uart_ui.nputs(ARRANDN("\033[2J\033[H"));
    System::uart_ui.nputs(ARRANDN(" " PROJECT_NAME "   " PROJECT_VERSION NEWLINE "\t - " PROJECT_DESCRIPTION NEWLINE "\t - compiled " __DATE__ " , " __TIME__ NEWLINE));

//    System::i2c1.tx_blocking(, data, size);

//    Task::BQ769x2_PROTOCOL_Test_V_Task(0);

    thing(0);

    while(true) {
        System::FailHard("reached end of main" NEWLINE);
    }
}
