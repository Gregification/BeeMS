/*
 * main.cpp
 *
 *  Created on: April 20, 2026
 *      Author: TURTL
 *      https://cataas.com/cat/says/accumulating
 */
/*
 * CANE boot loader
 *      target: mspm0g3519
 *
 * dev notes
 *  - for the love of god do not use IRQ's unless your willing to catch every edge case when swapping VT's and what not.
 *
 * references
 *  - Technical Data Sheet (TDS) : www.ti.com/lit/ug/slau846d/slau846d.pdf
 *      e.g: "[TDS.1.3/45]" -> section 1.3 (page 45) of the technical datasheet
 */

#include <stdint.h>
#include <stdbool.h>

#include <ti/driverlib/driverlib.h>

/*** IMPORTANT *********************************************************************/
// in the app, c&p the code as the very first things to run. sometimes the start up
//  scripts reset these values regardless what the boot loader does
/*
    SCB->VTOR = APP_START_ADDR; // set VT mem offset
*/

/*** project setup *************
 *  - this BL uses the first 4kB. application must start after that. edit linker cmd.
 *  - programmer must be instructed NOT to erase that the BL program location
 *      project > debug > 'mspm0 flash settings' > 'erase MAIN memory sectors by range'
 *          start:  0x1000
 *          end:    0x7FFFF // or what ever the flash range is for the chip
 *
 */

/***********************************************************************************/


/*** USR DEFINED ***************/

#define APP_START_ADDR 0x1000   // address of app vector table. must be >=BL_SIZE (see linker cmd)

/*******************************/

extern uint32_t BL_SIZE;

void startApp();

/** init peripherals used by BL */
void BL_init();
/** resets all peripherals modified by BL */
void BL_deinit();

int main(){

    if((uint32_t)&BL_SIZE > APP_START_ADDR) {
        // TODO: perform BL static failure blink code
        while(1);
    }

    startApp();
}

void BL_init() {

}

void BL_deinit() {

}

void startApp() {
    __disable_irq();

    // vector table info at [TDN.3.3.2/460]
    typedef  void (*VTHandler_f)(void);

    uint32_t * avt = (uint32_t *)APP_START_ADDR; // app vector table

    /*** validate app ***********/

    // is SP in valid ram
    if (avt[0] < 0x20200000 || avt[0] > 0x20220000) {
        // TODO: perform BL VT-RAM failure blink code
        while(1);
    }

    // is reset handle is valid flash
    if (avt[1] == 0xFFFFFFFF) {
        // TODO: perform BL VT-reset failure blink code
        while(1);
    }


    /*** launch app *************/

    __set_MSP(avt[0]); // set SP
    SCB->VTOR = APP_START_ADDR; // set VT mem offset

    volatile VTHandler_f app_reset = (VTHandler_f )avt[1];
    app_reset();

    while(1); // should never run
}
