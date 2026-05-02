/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 *
 *  This task is the interface with RapidSCADA.
 *  Achieved by a Modbus-TCP connection through the W5500.
 *
 *  RESOURCES:
 *      - Modbus-TCP : https://www.prosoft-technology.com/kb/assets/intro_modbustcp.pdf
 *          - TLDR: its just Modbus-RTU but sent over TCP
 *          - doc has a LOT of info. see Modbus-RTU doc for more on relevant stuff
 *          - "Modbus TCP/IP (also Modbus-TCP) is simply the Modbus RTU protocol
 *              with a TCP interface that runs on Ethernet." /4
 *      - Modbus-RTU : https://www.modbustools.com/modbus.html
 *          - doc has too-d-point packet breakdown
 *      - https://docs.google.com/drawings/d/1qxaCa3I_cCNk18BFZx2MLTmkAGG7i-K8t3E4OW4eXwk/edit?usp=sharing
 *
 * - in the name of professor Microwave, we will put all the code in one function
 * - probably reentrant but i wouldn't find out. easier to add-on multi-socket support
 *      if thats what your looking at.
 */

/*
 * TODO: look at later : https://www.tij.co.jp/jp/lit/ml/slyp847/slyp847.pdf
 */

#include <cstdint>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <Tasks/task_ethBridge.hpp>
#include "Core/system.hpp"
#include "Core/Board.hpp"
#include "Core/std alternatives/string.hpp"
#include "Middleware/W5500/socket.h"
#include "Middleware/W5500/wizchip_conf.h"


/*** setup ***************************************************/
System::UART::UART &            uart = System::UART::uart_ui;

/*** CAN ***************/
System::CANFD::CANFD &          can = System::CANFD::canFD0;

/*** wizchip setup ****/
System::SPI::SPI &              wiz_spi   = Board::Eth::spi;
System::GPIO::GPIO const &      wiz_cs    = Board::Eth::cs;
System::GPIO::GPIO const &      wiz_reset = Board::Eth::reset;
//System::SPI::SPI &              wiz_spi   = System::SPI::spi1;
//System::GPIO::GPIO const &      wiz_cs    = System::GPIO::PB3;
//System::GPIO::GPIO const &      wiz_reset = System::GPIO::PB2;
//System::SPI::SPI &              wiz_spi   = System::SPI::spi0; // workie
//System::GPIO::GPIO const &      wiz_cs    = System::GPIO::PA15;
//System::GPIO::GPIO const &      wiz_reset = System::GPIO::PB14;
uint16_t const                  modbusTCPPort = 502;   // arbitrary, must match RapidSCADA connection settings

uint8_t                         sockets[] = {0,1,2,3};

wiz_NetInfo netConfig = {
           .mac = {0xBE,0xEE,0xEE,0x00,0x00,0x00}, // arbitrary
//           .ip  = {192,168,1,2},
//           .sn  = {255,255,255,0},
           .ip  = {192,168,1,21},
           .sn  = {255,255,255,0},
//           .ip  = {169,254,0,1},
//           .sn  = {255,255,0,0},
           .gw  = {192,168,1,1},
           .dns = {8,8,8,8},
           .dhcp= NETINFO_STATIC
    };


/*************************************************************/


/** returns true on success */
bool setupWizchip();
/** prints human readable meaning of socket error code */
void wiz_print_sockerror(int8_t error);


/*** main ****************************************************/

void Task::ethBridge(void *){
    uart.nputs(ARRANDN("ethModbus_task start" NEWLINE));

    wiz_spi.setSCLKTarget(32e6);

    /*** W5500 init *****************/

    uart.nputs(ARRANDN(CLIHIGHLIGHT "awaiting W5500 response ..." CLIRESET NEWLINE));

    while(!setupWizchip()) {
        uart.nputs(ARRANDN(CLIERROR "non-fatal: W5500 init failed! retrying..." CLIRESET NEWLINE));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    wizchip_setnetinfo(&netConfig);
    {
        wiz_NetTimeout timeout = {
               .retry_cnt = 2,
               .time_100us = 1000,
            };
        wizchip_settimeout(&timeout);
    }


    {
        for(int i = 0; i < sizeof(sockets)/sizeof(sockets[0]); i++){
            vTaskDelay(pdMS_TO_TICKS(10));
            disconnect(sockets[i]);
            close(sockets[i]);
        }
    }

    /********************************/
    {
        wiz_NetInfo read;
        uart.nputs(ARRANDN(CLIRESET "reading back net config ... " NEWLINE));
        wizchip_getnetinfo(&read);

        while(!ALT::memcmp(&read, &netConfig, sizeof(netConfig))) {
            uart.nputs(ARRANDN(CLIRESET CLIERROR "W5500" CLINO " read" CLIERROR " config does not match" CLIYES " written" NEWLINE));

            uart.nputs(ARRANDN(CLIYES "IP: \t"));
            for(int i = 0; i < 4; i++){
                uart.putu32d(netConfig.ip[i]);
                uart.nputs(ARRANDN("."));
            }
            uart.nputs(ARRANDN(NEWLINE "SNM: \t"));
            for(int i = 0; i < 4; i++){
                uart.putu32d(netConfig.sn[i]);
                uart.nputs(ARRANDN("."));
            }
            uart.nputs(ARRANDN(NEWLINE "MAC: \t"));
            for(int i = 0; i < 6; i++){
                uart.putu32h(netConfig.mac[i]);
                uart.nputs(ARRANDN(" "));
            }

            uart.nputs(ARRANDN(CLINO NEWLINE));

            uart.nputs(ARRANDN("IP: \t"));
            for(int i = 0; i < 4; i++){
                uart.putu32d(read.ip[i]);
                uart.nputs(ARRANDN("."));
            }
            uart.nputs(ARRANDN(NEWLINE "SNM: \t"));
            for(int i = 0; i < 4; i++){
                uart.putu32d(read.sn[i]);
                uart.nputs(ARRANDN("."));
            }
            uart.nputs(ARRANDN(NEWLINE "MAC: \t"));
            for(int i = 0; i < 6; i++){
                uart.putu32h(read.mac[i]);
                uart.nputs(ARRANDN(" "));
            }


            wizchip_sw_reset();
            if(0 != wizchip_init(NULL,NULL))
                uart.nputs(ARRANDN("W5500 failed to reinit D:" NEWLINE)); // uh oh
            vTaskDelay(pdMS_TO_TICKS(1000));
            wizchip_getnetinfo(&read);
            uart.nputs(ARRANDN(NEWLINE));
        }

        uart.nputs(ARRANDN(CLIRESET NEWLINE CLIHIGHLIGHT"IP: \t"));
        for(int i = 0; i < 4; i++){
            uart.putu32d(read.ip[i]);
            uart.nputs(ARRANDN("."));
        }
        uart.nputs(ARRANDN(NEWLINE "SNM: \t"));
        for(int i = 0; i < 4; i++){
            uart.putu32d(read.sn[i]);
            uart.nputs(ARRANDN("."));
        }
        uart.nputs(ARRANDN(NEWLINE "MAC: \t"));
        for(int i = 0; i < 6; i++){
            uart.putu32h(read.mac[i]);
            uart.nputs(ARRANDN(" "));
        }

        uart.nputs(ARRANDN(CLIRESET NEWLINE));
    }

    // broadcast to ensure device is known to network equipment
    {
        SOCKET s = sockets[0];
        int8_t error = socket(s, Sn_MR_UDP, 123, SF_IO_NONBLOCK);
        if(error == s) {
            uint8_t addr[4];
            ALT::memcpy(netConfig.ip, addr, sizeof(addr));
            addr[3] = 255;
            sendto(s, addr, 1, addr, 1234);
        }
        close(s);
    }

    while(true) {
//        for(uint8_t i = 0; i < sizeof(sockets)/sizeof(sockets[0]); i++){
//            checkSocket(sockets[i], &rxbuf, &txbuf);
//            vTaskDelay(pdMS_TO_TICKS(3));// some delay for the thingy-ma-jig to do its stuff
//        }
//        checkCAN(&rxbuf, &txbuf);
    }

    /********************************/
    /* we just wait for a connection, only support single connection.
     * 1. init socket
     * 2. setup socket to listen for incoming tcp
     * 3. wait for incoming connection
     * 4. receive packet
     * 5. process or forward request
     * 6(7). if necessary, transmit back data
     * 7. go to step 4
     *
     * - to add support for multiple connections just make more sockets.
     * - if a error happens reset connection and restart from step one
     */


    System::FailHard("ethModbus_task ended" NEWLINE);
    vTaskDelete(NULL);
}


/*** wizchip setup *******************************************/

void wiz_select(void)       { wiz_spi.takeResource(0); wiz_cs.set();  }
void wiz_deselect(void)     { wiz_cs.clear(); wiz_spi.giveResource(); }
uint8_t wiz_read_byte(void) {
    uint8_t rx;
    wiz_spi.transfer(NULL, &rx, 1);
    while(wiz_spi.isBusy());
    return rx;
}
void wiz_write_byte(uint8_t b) {
    wiz_spi.transfer(&b, NULL, 1);
    while(wiz_spi.isBusy());
}
void wiz_read_burst(uint8_t *buf, uint16_t len) {
    wiz_spi.transfer(NULL, buf, len);
    while(wiz_spi.isBusy());
}
void wiz_write_burst(uint8_t *buf, uint16_t len) {
    wiz_spi.transfer(buf, NULL, len);
    while(wiz_spi.isBusy());
}
void wiz_enter_critical(){ } // DO NOT USE FRERTOS TASK CRITICAL!
void wiz_exit_critical(){ }
void wiz_print_sockerror(int8_t error) {
    switch(error){
        case SOCKERR_SOCKNUM:   uart.nputs(ARRANDN("SOCKERR_SOCKNUM")); break;
        case SOCKERR_SOCKMODE:  uart.nputs(ARRANDN("SOCKERR_SOCKMODE")); break;
        case SOCKERR_SOCKFLAG:  uart.nputs(ARRANDN("SOCKERR_SOCKFLAG")); break;
        case SOCKERR_SOCKCLOSED:uart.nputs(ARRANDN("SOCKERR_SOCKCLOSED")); break;
        case SOCKERR_SOCKINIT:  uart.nputs(ARRANDN("SOCKERR_SOCKINIT")); break;
        case SOCKERR_SOCKOPT:   uart.nputs(ARRANDN("SOCKERR_SOCKOPT")); break;
        case SOCKERR_SOCKSTATUS:uart.nputs(ARRANDN("SOCKERR_SOCKSTATUS")); break;
        case SOCKERR_DATALEN:   uart.nputs(ARRANDN("SOCKERR_DATALEN")); break;
        case SOCKERR_PORTZERO:  uart.nputs(ARRANDN("SOCKERR_PORTZERO")); break;
        case SOCKERR_TIMEOUT:   uart.nputs(ARRANDN("SOCKERR_TIMEOUT")); break;
        case SOCK_BUSY:         uart.nputs(ARRANDN("SOCK_BUSY")); break;
        default:                uart.nputs(ARRANDN("no switch case")); break;
    }
}

bool setupWizchip() {
    reg_wizchip_spi_cbfunc(wiz_read_byte, wiz_write_byte);
    reg_wizchip_spiburst_cbfunc(wiz_read_burst, wiz_write_burst);
    reg_wizchip_cris_cbfunc(wiz_enter_critical, wiz_exit_critical);
    reg_wizchip_cs_cbfunc(wiz_select, wiz_deselect);

    DL_GPIO_initDigitalOutputFeatures(
            wiz_cs.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_clearPins(GPIOPINPUX(wiz_cs));
    DL_GPIO_enableOutput(GPIOPINPUX(wiz_cs));
    wiz_cs.clear();

    wiz_select();
    wiz_write_byte(0); // to reset the SCLK pin
    wiz_deselect();

    DL_GPIO_initDigitalOutputFeatures(
            wiz_reset.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_enableOutput(GPIOPINPUX(wiz_reset));


    // reset device
    //  also gives CS line time to stabilize
    wiz_reset.clear();
    vTaskDelay(pdMS_TO_TICKS(10));
    wiz_reset.set();
    vTaskDelay(pdMS_TO_TICKS(10));
    wiz_reset.clear();
    vTaskDelay(pdMS_TO_TICKS(10));

    return 0 == wizchip_init(NULL,NULL);
}
