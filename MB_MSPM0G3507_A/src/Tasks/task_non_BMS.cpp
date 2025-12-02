/*
 * fiddle.cpp
 *
 *  Created on: Jun 6, 2025
 *      Author: FSAE
 */

/*
 * TODO: look at later : https://www.tij.co.jp/jp/lit/ml/slyp847/slyp847.pdf
 */

#include <cstdint>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>

#include "Tasks/task_non_BMS.hpp"

#include "Core/system.hpp"
#include "Core/std alternatives/string.hpp"
#include "Middleware/W5500/socket.h"
#include "Middleware/W5500/wizchip_conf.h"

/*** wixchip setup *******************************************/

auto &wiz_spi   = System::spi1;
auto &wiz_cs    = System::GPIO::PA8;
auto &wiz_reset = System::GPIO::PA15;

uint16_t const modbusTCPPort = 502;   // arbitrary, must match RapidSCADA settings
uint8_t const socketNum = 0;          // [0,8], arbitrary

wiz_NetInfo netConfig = {
           .mac = {0xBE,0xEE,0xEE,0x00,0x00,0x00}, // arbitrary
           .ip  = {192,168,1,252},
           .sn  = {255,255,255,0},
           .gw  = {192,168,1,1},
           .dns = {8,8,8,8},
           .dhcp= NETINFO_STATIC
    };

/** returns true on success */
bool setupWizchip();
/** prints human readable meaning of socket error code */
void wiz_print_sockerror(int8_t error);


/*** main ****************************************************/

void Task::non_BMS_ethModbus_task(void *){
    System::uart_ui.nputs(ARRANDN("non_BMS_ethModbus_task start" NEWLINE));

    wiz_spi.setSCLKTarget(25e6);

    /*** W5500 init *****************/
    if(!setupWizchip())
        System::FailHard("W5500 init failed!");

    wizchip_setnetinfo(&netConfig);
    {
        wiz_NetTimeout timeout = {
               .retry_cnt = 3,
               .time_100us = 2500,
            };
        wizchip_settimeout(&timeout);
    }

    /********************************/
    System::uart_ui.nputs(ARRANDN(NEWLINE "IP: \t"));
    for(int i = 0; i < 4; i++){
        System::uart_ui.putu32d(netConfig.ip[i]);
        System::uart_ui.nputs(ARRANDN("."));
    }
    System::uart_ui.nputs(ARRANDN(NEWLINE "SNM: \t"));
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


    /********************************/
    /* we just wait for a connection, only support single connection. not sure how to get the w5500 to both listen and receive packets
     * listen is blocking only. probably could just do it anyways and be fine but thats extra work. someone else can do it later
     */

    int8_t error;
    SOCKET sn;
    bool reset;

    while(true){
        reset = false;
        sn = socketNum;

        // init socket
        switch(error = socket(sn, Sn_MR_TCP, modbusTCPPort, SF_IO_NONBLOCK)){
            default:
                if(error == sn)
                    // yippie
                    break;

                System::uart_ui.nputs(ARRANDN("W5500 unknown response to socket!" NEWLINE));

            case SOCKERR_SOCKMODE:
            case SOCKERR_SOCKFLAG:
            case SOCKERR_SOCKNUM:
                // failed to init socket, retry
                System::uart_ui.nputs(ARRANDN(CLIERROR "Modbus TCP: failed to init socket : " ));
                wiz_print_sockerror(error);
                System::uart_ui.nputs(ARRANDN(CLIRESET NEWLINE));
                vTaskDelay(pdMS_TO_TICKS(1e3));
                close(sn);
                continue;
        }

        // establish incomming connection
        error = listen(sn);
        switch(error) {
            default:
            case SOCKERR_SOCKINIT:
                System::uart_ui.nputs(ARRANDN("SOCKERR_SOCKINIT , "));
            case SOCKERR_SOCKCLOSED:
                System::uart_ui.nputs(ARRANDN("SOCKERR_SOCKCLOSED , "));
                System::uart_ui.nputs(ARRANDN("Modbus TCP listen connection failed." NEWLINE));

                // bad connection, ignore
                continue;

            case SOCK_OK:
                // yippie!
                break;
        }

        System::uart_ui.nputs(ARRANDN(CLIHIGHLIGHT "Modbus listening for TCP connection on port: " ));
        System::uart_ui.putu32d(modbusTCPPort);
        System::uart_ui.nputs(ARRANDN(CLIRESET NEWLINE));


        while((error = getSn_SR(sn)) != SOCK_ESTABLISHED){
            switch(error){
                // cases found at w5500.h/480

                case SOCK_LISTEN:
                    // keep waiting
//                    System::uart_ui.nputs(ARRANDN("sock listening..." NEWLINE));
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;

                default:
                    // restart
                    reset = true;
                    break;
            }
            if(reset)
                break;
        }
        if(reset) continue;

        static uint8_t buf[20];
        static_assert(sizeof(buf) <= UINT16_MAX);

        // process packets
        while(!reset){
            int32_t status = recv(sn, ARRANDN(buf));

            // is data available?
            switch(status){
                case SOCK_BUSY:
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                case SOCKERR_DATALEN:
                    System::uart_ui.nputs(ARRANDN("Modbus: zero data length" NEWLINE));
                    reset = true;
                    continue;
                case SOCKERR_SOCKNUM:
                    System::uart_ui.nputs(ARRANDN("Modbus: Invalid socket number" NEWLINE));
                    reset = true;
                    continue;
                case SOCKERR_SOCKMODE:
                    System::uart_ui.nputs(ARRANDN("Modbus: Invalid operation in the socket" NEWLINE));
                    reset = true;
                    continue;
                case SOCKERR_SOCKSTATUS:
                    System::uart_ui.nputs(ARRANDN("Modbus: Invalid socket status for socket operation" NEWLINE));
                    reset = true;
                    continue;
                default:
                    if(status >= 0 && status <= sizeof(buf))
                        // yippie
                        break;

                    System::uart_ui.nputs(ARRANDN("Modbus: unknown receive status!? -> "));
                    System::uart_ui.put32d(status);
                    System::uart_ui.nputs(ARRANDN(CLIRESET NEWLINE));
                    reset = true;
                    continue;
            }

            // process packet
            System::uart_ui.nputs(ARRANDN("Modbus: dump packet" NEWLINE));
            for(uint16_t i = 0; i < sizeof(buf); i++){
                System::uart_ui.nputs(ARRANDN(" \t"));
                System::uart_ui.putu32h(buf[i]);
                if(i % 8 == 0)
                    System::uart_ui.nputs(ARRANDN(NEWLINE));
            }
            System::uart_ui.nputs(ARRANDN(NEWLINE));
        }

        vTaskDelay(pdMS_TO_TICKS(10e3));
    }

    System::FailHard("non_BMS_functions_task ended" NEWLINE);
    vTaskDelete(NULL);
}


/*** wizchip setup *******************************************/

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
void wiz_enter_critical(){ } // DO NOT USE FRERTOS TASK CRITICAL! just claim the spi bus
void wiz_exit_critical(){ }
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
    wiz_write_byte(0); // to reset the SCLK pin

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

    if(wizchip_init(NULL,NULL)) {
        System::uart_ui.nputs(ARRANDN(CLIERROR "failed wizchip init chip" CLIRESET NEWLINE));
        return false;
    } else {
        System::uart_ui.nputs(ARRANDN(CLIGOOD "wizchip init-ed chip" CLIRESET NEWLINE));
        return true;
    }
}
