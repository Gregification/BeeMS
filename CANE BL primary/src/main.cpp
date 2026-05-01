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
 *  - BLS and BC memory edits have been disabled in this CCS project
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

#define BV(X) (1 << (X))


/*** USR DEFINED ***************/

#define APP_START_ADDR 0x1000   // address of app vector table. must be >=BL_SIZE (see linker cmd)


/*******************************/

extern uint32_t BL_SIZE;

#define LED_PORT    GPIOA
#define LED_PIN     BV(0)
#define LED_PINCM   IOMUX_PINCM1

void startApp();

/** init peripherals used by BL */
void BL_init();
/** resets all peripherals modified by BL */
void BL_deinit();

void delaymS(uint32_t ms);
void blinkError();

int main(){
    BL_init();

    if((uint32_t)&BL_SIZE > APP_START_ADDR) {
        blinkError();
    }

    startApp();
}

void BL_init() {
    DL_GPIO_disablePower(LED_PORT);
    DL_GPIO_reset(LED_PORT);
    DL_GPIO_enablePower(LED_PORT);
    delay_cycles(16);

    DL_GPIO_initDigitalOutput(LED_PINCM);
    DL_GPIO_enableOutput(LED_PORT, LED_PIN);
    DL_GPIO_initDigitalOutputFeatures(
            LED_PINCM,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_clearPins(LED_PORT, LED_PIN);
    DL_GPIO_enableOutput(LED_PORT, LED_PIN);
}

void BL_deinit() {
    DL_GPIO_disableOutput(LED_PORT, LED_PIN);

    DL_GPIO_disablePower(LED_PORT);
    DL_GPIO_reset(LED_PORT);
}

void startApp() {
    volatile uint32_t * avt = (uint32_t *)APP_START_ADDR; // app vector table

    /*** validate app ***********/

    // is SP in valid ram
    if (avt[0] < 0x20200000 || avt[0] > 0x20220000) {
        blinkError();
    }

    // is reset handle is valid flash
    if (avt[1] == 0xFFFFFFFF) {
        blinkError();
    }

    /*** launch app *************/
    __disable_irq();
    BL_deinit();

    __set_MSP(avt[0]); // set SP
    SCB->VTOR = APP_START_ADDR; // set VT mem offset

    // vector table info at [TDN.3.3.2/460]
    typedef  void (*VTHandler_f)(void);

    void (*app_reset)(void) = (void (*)(void))avt[1];
    app_reset();

    blinkError(); // should never run
}

void delaymS(uint32_t ms) {
    delay_cycles(ms * 32e3); // default power on clock is 32e6
}

void blinkError() {
    // i am not decoding a led live. if this is ever seen just know its a BL error.

    while(1) {
        for(int i = 0; i < 10; i++) {
            DL_GPIO_togglePins(LED_PORT, LED_PIN);
            delaymS(30);
        }
        for(int i = 0; i < 7; i++) {
            DL_GPIO_togglePins(LED_PORT, LED_PIN);
            delaymS(100);
        }
    }
}
