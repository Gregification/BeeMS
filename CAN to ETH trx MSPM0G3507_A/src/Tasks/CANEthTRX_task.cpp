/*
 * CANEthTRXUI_task.cpp
 *
 *  Created on: Oct 3, 2025
 *      Author: turtl
 */

#include <Tasks/CANEthTRX_task.hpp>

#include <stdint.h>

#include "Core/system.hpp"
#include "Middleware/W5500/socket.h"
#include "Middleware/W5500/wizchip_conf.h"

/*** wizchip setup **********************************************/

auto &wiz_spi   = System::spi0;
auto &wiz_cs    = System::GPIO::PA8;
auto &wiz_reset = System::GPIO::PA15;

uint8_t tx_target_IP[4] = {192,168,1,255};

wiz_NetInfo netConfig = {
           .mac = {0xBE,0xEE,0xEE,0x00,0x00,0x00},
           .ip  = {192,168,1,252},
           .sn  = {255,255,255,0},
           .gw  = {192,168,1,1},
           .dns = {8,8,8,8},
           .dhcp= NETINFO_STATIC
    };

void wiz_select(void)       { wiz_cs.set();    } // if spi runs at 32MHz no need for delay
void wiz_deselect(void)     { wiz_cs.clear();  }
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
void wiz_enter_critical(){}
void wiz_exit_critical(){}
void wiz_print_sockerror(int8_t error) {
    switch(error){
        case SOCKERR_SOCKNUM:   System::uart_ui.nputs(ARRANDN("SOCKERR_SOCKNUM")); break;
        case SOCKERR_SOCKMODE:  System::uart_ui.nputs(ARRANDN("SOCKERR_SOCKMODE")); break;
        case SOCKERR_SOCKFLAG:  System::uart_ui.nputs(ARRANDN("SOCKERR_SOCKFLAG")); break;
        case SOCKERR_SOCKCLOSED:System::uart_ui.nputs(ARRANDN("SOCKERR_SOCKCLOSED")); break;
        case SOCKERR_SOCKINIT:  System::uart_ui.nputs(ARRANDN("SOCKERR_SOCKINIT")); break;
        case SOCKERR_SOCKOPT:   System::uart_ui.nputs(ARRANDN("SOCKERR_SOCKOPT")); break;
        case SOCKERR_SOCKSTATUS:System::uart_ui.nputs(ARRANDN("SOCKERR_SOCKSTATUS")); break;
        case SOCKERR_DATALEN:   System::uart_ui.nputs(ARRANDN("SOCKERR_DATALEN")); break;
        case SOCKERR_PORTZERO:  System::uart_ui.nputs(ARRANDN("SOCKERR_PORTZERO")); break;
        case SOCKERR_TIMEOUT:   System::uart_ui.nputs(ARRANDN("SOCKERR_TIMEOUT")); break;
        case SOCK_BUSY:         System::uart_ui.nputs(ARRANDN("SOCK_BUSY")); break;
        default:                System::uart_ui.nputs(ARRANDN("no switch case")); break;
    }
}


/****************************************************************/

void Task::CAN_Eth_TRX_UI_task(void *){
    System::uart_ui.nputs(ARRANDN("CAN_Eth_TRX_UI_task start" NEWLINE));

    /*** wizchip init ***********************************************/

    wiz_spi.setSCLKTarget(5e6);

    reg_wizchip_spi_cbfunc(wiz_read_byte, wiz_write_byte);
    reg_wizchip_spiburst_cbfunc(wiz_read_burst, wiz_write_burst);
    reg_wizchip_cris_cbfunc(wiz_enter_critical, wiz_exit_critical);
    reg_wizchip_cs_cbfunc(wiz_select, wiz_deselect);

    int8_t error;

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
    wiz_write_byte(0); // to reset the SCLK pin

    DL_GPIO_initDigitalOutputFeatures(
            wiz_reset.iomux,
            DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE,
            DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
            DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_LOW,
            DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
        );
    DL_GPIO_enableOutput(GPIOPINPUX(wiz_reset));
    wiz_reset.clear();
    delay_cycles(System::CLK::CPUCLK / 10);
    wiz_reset.set();
    delay_cycles(System::CLK::CPUCLK / 10);
    wiz_reset.clear();
    delay_cycles(System::CLK::CPUCLK / 10);

//    while(1){
//        wiz_select();
//        wiz_write_byte(1);
//        wiz_write_byte(2);
//        wiz_write_byte(3);
//        uint8_t ret = wiz_read_byte();
//        wiz_deselect();
//    }

    if(wizchip_init(NULL,NULL))
        System::uart_ui.nputs(ARRANDN("failed wizchip init chip" NEWLINE));
    else
        System::uart_ui.nputs(ARRANDN("wizchip init-ed chip" NEWLINE));

    SOCKET sn = 0;
    if((error = socket(sn, Sn_MR_UDP, 2080, 0)) != sn){
        System::uart_ui.nputs(ARRANDN("failed wizchip init socket" NEWLINE "\t"));
        wiz_print_sockerror(error);
        System::uart_ui.nputs(ARRANDN(NEWLINE));
    } else
        System::uart_ui.nputs(ARRANDN("wizchip init-ed socket" NEWLINE));

    wizchip_setnetinfo(&netConfig);

    System::uart_ui.nputs(ARRANDN(NEWLINE "IP:\t"));
    for(int i = 0; i < 4; i++){
        System::uart_ui.putu32d(netConfig.ip[i]);
        System::uart_ui.nputs(ARRANDN("."));
    }
    System::uart_ui.nputs(ARRANDN(NEWLINE "SN: \t"));
    for(int i = 0; i < 4; i++){
        System::uart_ui.putu32d(netConfig.sn[i]);
        System::uart_ui.nputs(ARRANDN("."));
    }
    System::uart_ui.nputs(ARRANDN(NEWLINE "MAC: \t"));
    for(int i = 0; i < 6; i++){
        System::uart_ui.putu32h(netConfig.mac[i]);
        System::uart_ui.nputs(ARRANDN(" "));
    }
    System::uart_ui.nputs(ARRANDN(NEWLINE));

    /****************************************************************/

    DL_MCAN_RxFIFOStatus rx_can;
    while(1){
        // RX CAN
        rx_can.num = DL_MCAN_RX_FIFO_NUM_0;
        DL_MCAN_getRxFIFOStatus(CANFD0, &rx_can);

        if(rx_can.fillLvl != 0) { // if not empty
            DL_MCAN_RxBufElement e;
            DL_MCAN_readMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, 0, rx_can.num, &e);
            DL_MCAN_writeRxFIFOAck(CANFD0, rx_can.num, rx_can.getIdx);

            uint32_t id;
            if(e.xtd)   id = e.id;
            else        id = (e.id & 0x1FFC'0000) >> 18;

            System::uart_ui.nputs(ARRANDN("rx CAN-ID: "));
            System::uart_ui.putu32d(id);
            System::uart_ui.nputs(ARRANDN(" ... "));

            if((error = sendto(sn, e.data, e.dlc, tx_target_IP, 33512)) != e.dlc){
                System::uart_ui.nputs(ARRANDN("failed TX to wizchip. \t"));
                wiz_print_sockerror(error);
            }
            else
                System::uart_ui.nputs(ARRANDN("success TX"));

            System::uart_ui.nputs(ARRANDN(NEWLINE));
        }

    }

    close(sn);


    System::FailHard("CAN_Eth_TRX_UI_task ended");
    vTaskDelete(NULL);
}
