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
#include <Middleware/W5500/wizchip_conf.h>

/*** wizchip setup *******************************************/
auto& wiz_spi   = System::spi0;
auto& wiz_cs    = System::GPIO::PA4;

void wiz_select(void)       { wiz_cs.set();     }
void wiz_deselect(void)     { wiz_cs.clear();   }
uint8_t wiz_read_byte(void) {
    uint8_t rx;
    wiz_spi.transfer_blocking(NULL, &rx, 1);
    return rx;
}
void wiz_write_byte(uint8_t b) {
    wiz_spi.transfer_blocking(&b, NULL, 1);
}
void wiz_read_burst(uint8_t *buf, uint16_t len) {
    wiz_spi.transfer_blocking(NULL, buf, len);
}
void wiz_write_burst(uint8_t *buf, uint16_t len) {
    wiz_spi.transfer_blocking(buf, NULL, len);
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

void Task::ethernetw5500_test(void *){
    System::uart_ui.nputs(ARRANDN("ethernetw5500_test start" NEWLINE));

    // wizz chip
        reg_wizchip_spi_cbfunc(wiz_read_byte, wiz_write_byte);
        reg_wizchip_spiburst_cbfunc(wiz_read_burst, wiz_write_burst);
        reg_wizchip_cris_cbfunc(wiz_enter_critical, wiz_exit_critical);
        reg_wizchip_cs_cbfunc(wiz_select, wiz_deselect);

        int8_t error;

        DL_GPIO_initDigitalOutputFeatures(
                wiz_cs.iomux,
                DL_GPIO_INVERSION::DL_GPIO_INVERSION_ENABLE,
                DL_GPIO_RESISTOR::DL_GPIO_RESISTOR_NONE,
                DL_GPIO_DRIVE_STRENGTH::DL_GPIO_DRIVE_STRENGTH_HIGH,
                DL_GPIO_HIZ::DL_GPIO_HIZ_DISABLE
            );
        DL_GPIO_clearPins(GPIOPINPUX(wiz_cs));
        DL_GPIO_enableOutput(GPIOPINPUX(wiz_cs));
        wiz_cs.clear();
        wiz_spi.setSCLKTarget(1e6);

        System::uart_ui.nputs(ARRANDN("start" NEWLINE));
        if(wizchip_init(NULL,NULL))
            System::uart_ui.nputs(ARRANDN("failed init chip" NEWLINE));
        else
            System::uart_ui.nputs(ARRANDN("init-ed chip" NEWLINE));

        SOCKET sn = 0;
        if((error = socket(sn, Sn_MR_UDP, 2080, 0)) != sn){
            System::uart_ui.nputs(ARRANDN("failed init socket" NEWLINE "\t"));
            wiz_print_sockerror(error);
            System::uart_ui.nputs(ARRANDN(NEWLINE));
        } else
            System::uart_ui.nputs(ARRANDN("init-ed socket" NEWLINE));

        {
            wiz_NetInfo netinfo = {
                   .mac = {0xBE,0xEE,0xEE,0x00,0x00,0x00},
                   .ip  = {192,168,1,211},
                   .sn  = {255,255,255,0},
                   .gw  = {192,168,1,1},
                   .dns = {8,8,8,8},
                   .dhcp= NETINFO_STATIC
            };
            wizchip_setnetinfo(&netinfo);
        }

        delay_cycles(36e6);
        char arr[]    = "12345678910111213141516171819202122232425262728293031323334353637383940";
        uint8_t ip[]    = {192,168,1,134};
        if((error = sendto(sn, (uint8_t *)arr, sizeof(arr), ip, 42069)) != sizeof(arr)){
            System::uart_ui.nputs(ARRANDN("failed send-to" NEWLINE));
            System::uart_ui.nputs(ARRANDN("\t"));
            wiz_print_sockerror(error);
        }
        else
            System::uart_ui.nputs(ARRANDN("send-to-ed" NEWLINE));

        close(sn);

    System::uart_ui.nputs(ARRANDN("ethernetw5500_test end" NEWLINE));
}

