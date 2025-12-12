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
#include <Tasks/task_ethModbus.hpp>

#include "Core/system.hpp"
#include "Core/std alternatives/string.hpp"
#include "Core/Networking/Modbus.hpp"
#include "Core/Networking/MasterModbusRegisters.hpp"
#include "Middleware/W5500/socket.h"
#include "Middleware/W5500/wizchip_conf.h"


/*** wixchip setup *******************************************/

System::SPI::SPI         &wiz_spi   = System::spi1;
System::GPIO::GPIO const &wiz_cs    = System::GPIO::PA8;
System::GPIO::GPIO const &wiz_reset = System::GPIO::PA15;

uint16_t const modbusTCPPort = 502;   // arbitrary, must match RapidSCADA connection settings
uint8_t const socketNum = 0;          // [0,8], arbitrary, socket must not be used elsewhere

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

System::UART::UART &uart = System::uart_ui;

void Task::ethModbus_task(void *){
    uart.nputs(ARRANDN("ethModbus_task start" NEWLINE));

//    do {
//        DL_MCAN_TxBufElement txmsg = {
//                .id     = 0x1,      // CAN id, 11b->[28:18], 29b->[] when using 11b can id
//                .rtr    = 0,        // 0: data frame, 1: remote frame
//                .xtd    = 1,        // 0: 11b id, 1: 29b id
//                .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
//                .dlc    = 3,        // data byte count, see DL comments
//                .brs    = 0,        // 0: no bit rate switching, 1: yes brs
//                .fdf    = 0,        // FD format, 0: classic CAN, 1: CAN FD format
//                .efc    = 0,        // 0: dont store Tx events, 1: store
//                .mm     = 0x3,      // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
//            };
//        txmsg.data[0] = 6;
//        txmsg.data[1] = 7;
//        txmsg.dlc = 2;
//
//        DL_MCAN_TxFIFOStatus tf;
//        DL_MCAN_getTxFIFOQueStatus(CANFD0, &tf);
//
//        uint32_t bufferIndex = tf.putIdx;
//        uart.nputs(ARRANDN("TX from buffer "));
//        uart.putu32d(bufferIndex);
//        uart.nputs(ARRANDN("" NEWLINE));
//
//        DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, bufferIndex, &txmsg);
//        DL_MCAN_TXBufAddReq(CANFD0, tf.getIdx);
//
//        vTaskDelay(pdMS_TO_TICKS(400));
//    } while(1);

    wiz_spi.setSCLKTarget(25e6);

    /*** W5500 init *****************/

    while(!setupWizchip()) {
        uart.nputs(ARRANDN(CLIERROR "non-fatal: W5500 init failed!" CLIRESET NEWLINE));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    wizchip_setnetinfo(&netConfig);
    {
        wiz_NetTimeout timeout = {
               .retry_cnt = 3,
               .time_100us = 2500,
            };
        wizchip_settimeout(&timeout);
    }


    /********************************/
    uart.nputs(ARRANDN(CLIHIGHLIGHT "IP: \t"));
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
    uart.nputs(ARRANDN(CLIRESET NEWLINE));


    /********************************/
    /* we just wait for a connection, only support single connection.
     * 1. init socket
     * 2. setup socket to listen for incoming tcp
     * 3. wait for incoming connection
     * 4. receive packet
     * 5. process request
     * 6(7). if necessary, transmit back data
     * 7. go to step 4
     *
     * - to add support for multiple connections just make more sockets.
     * - if a error happens reset connection and restart from step one
     */

    int8_t error;
    SOCKET sn;
    bool reset;

    while(true){
        reset = false;
        sn = socketNum;

        /*** init socket ****************/

        switch(error = socket(sn, Sn_MR_TCP, modbusTCPPort, SF_IO_NONBLOCK)){
            default:
                if(error == sn)
                    // yippie
                    break;

                uart.nputs(ARRANDN("W5500 unknown response to socket!" NEWLINE));

            case SOCKERR_SOCKMODE:
            case SOCKERR_SOCKFLAG:
            case SOCKERR_SOCKNUM:
                // failed to init socket, retry
                uart.nputs(ARRANDN(CLIERROR "Modbus TCP: failed to init socket : " ));
                wiz_print_sockerror(error);
                uart.nputs(ARRANDN(CLIRESET NEWLINE));
                vTaskDelay(pdMS_TO_TICKS(1e3));
                close(sn);
                continue;
        }


        /*** setup socket to listen *****/

        error = listen(sn);
        switch(error) {
            default:
            case SOCKERR_SOCKINIT:
                uart.nputs(ARRANDN("SOCKERR_SOCKINIT , "));
            case SOCKERR_SOCKCLOSED:
                uart.nputs(ARRANDN("SOCKERR_SOCKCLOSED , "));
                uart.nputs(ARRANDN("Modbus: TCP listen connection failed." NEWLINE));

                // bad connection, ignore
                continue;

            case SOCK_OK:
                // yippie!
                break;
        }


        /*** wait for incoming connection ***/

        uart.nputs(ARRANDN("Modbus: listening for TCP connection on port: " ));
        uart.putu32d(modbusTCPPort);
        uart.nputs(ARRANDN(CLIRESET NEWLINE));

        while((error = getSn_SR(sn)) != SOCK_ESTABLISHED){ // get socket status
            switch(error){
                // cases found at w5500.h/480

                case SOCK_LISTEN:
                    // keep waiting
                    vTaskDelay(pdMS_TO_TICKS(100));
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


        /*** receive packets ******************/

        uart.nputs(ARRANDN("Modbus: accepted incoming TCP connection on port: " ));
        uart.putu32d(modbusTCPPort);
        uart.nputs(ARRANDN(CLIRESET NEWLINE));

        static uint8_t rxbuf[60]; // 260B MAX standard Modbus-TCP len (MBAP + PDU), we can get away with smaller for 99% of stuff
        static uint8_t txbuf[70]; // see comment above. idk. what do u want from me. go find the campus cat. ("mmmm" (Microwave imitation))
        static constexpr uint8_t * txend = txbuf + sizeof(txbuf);

        static_assert(sizeof(rxbuf) <= UINT16_MAX);
        static_assert(sizeof(txbuf) <= UINT16_MAX);


        while(!reset){
            // rx packet
            int32_t status = recv(sn, ARRANDN(rxbuf));
            switch(status){
                static uint8_t cycles = 1; // 'exponential'(allegedly) back off wait times

                case SOCK_BUSY:
                    if(cycles < 20) // back off max cap at 20mS
                        cycles++;

                    vTaskDelay(pdMS_TO_TICKS(cycles));
                    continue;
                case SOCKERR_DATALEN:
                    uart.nputs(ARRANDN(CLIWARN "Modbus: zero data length" CLIRESET NEWLINE));
                    reset = true;
                    continue;
                case SOCKERR_SOCKNUM:
                    uart.nputs(ARRANDN(CLIWARN "Modbus: Invalid socket number" CLIRESET NEWLINE));
                    reset = true;
                    continue;
                case SOCKERR_SOCKMODE:
                    uart.nputs(ARRANDN(CLIWARN "Modbus: Invalid operation in the socket" CLIRESET NEWLINE));
                    reset = true;
                    continue;
                case SOCKERR_SOCKSTATUS:
                    uart.nputs(ARRANDN(CLIWARN "Modbus: Invalid socket status for socket operation" CLIRESET NEWLINE));
                    reset = true;
                    continue;
                default:
                    if(status >= 0 && status <= sizeof(rxbuf)){
                        // yippie

                        cycles = 0; // reset wait
                        break;
                    }

                    uart.nputs(ARRANDN(CLIERROR "Modbus: unknown receive status!? -> "));
                    uart.put32d(status);
                    uart.nputs(ARRANDN(CLIRESET NEWLINE));
                    reset = true;
                    continue;
            }


            /*** handle packet **************************************/

            uint16_t const rxlen = status;
            void const * const rxend = (void*)(rxbuf + rxlen);

            // dump packet
//            uart.nputs(ARRANDN("Modbus: dump packet" NEWLINE));
//            for(uint16_t i = 0; i < rxlen; i++){
//                uart.nputs(ARRANDN(" \t"));
//                uart.putu32h(rxbuf[i]);
//                if(i % 8 == 0 && i != 0)
//                    uart.nputs(ARRANDN(NEWLINE));
//            }
//            uart.nputs(ARRANDN(NEWLINE));

            do {
                // make life simple
                using namespace System;
                using namespace Networking::Modbus;

                MBAPHeader const * rxheader = reinterpret_cast<MBAPHeader const *>(rxbuf);
                ADUPacket const * rxadu     = rxheader->adu;
                MBAPHeader * txheader       = reinterpret_cast<MBAPHeader *>(txbuf);
                ADUPacket * txadu           = txheader->adu;


                /*** validation *****************/

                if(rxadu >= rxend) // is under sized?
                    break; // ignore packet

                if(ntoh16(rxheader->protocolID) != MBAPHeader::PROTOCOL_ID_MODBUS) // is protocol not Modbus ?
                    break; // ignore packet

                uart.nputs(ARRANDN("unitID: "));
                uart.putu32h(rxheader->unitID);
                uart.nputs(ARRANDN(NEWLINE));

                /*** process packet *************/

                switch (rxadu->func) {
                    case Function::R_COILS:
                        uart_ui.nputs(ARRANDN("R_COILS unhandled" NEWLINE));
                        break;

                    case Function::R_DISRETE_INPUTS:
                        uart_ui.nputs(ARRANDN("R_DISRETE_INPUTS unhandled" NEWLINE));
                        break;

                    case Function::R_HOLDING_REGS: {
//                            uart_ui.nputs(ARRANDN("R_HOLDING_REGS" NEWLINE));
                            F_Range_REQ const * query = reinterpret_cast<F_Range_REQ const *>(rxadu->data);
                            F_Range_RES * resp = reinterpret_cast<F_Range_RES *>(txadu->data);

                            /*** validation ***********/

                            if((sizeof(*query) + 2) > ntoh16(rxheader->len))// is "query" struct in packet out of bounds ?  // +2 = [unit id] + [function]
                                break; // ignore packet


                            /*** response *************/

                            *txheader   = *rxheader;
                            *txadu      = *rxadu;
                            resp->byteCount = 0;

                            for(uint16_t i = 0; i < ntoh16(query->len); i++){ // for each requested address
                                if((void*)(resp->values + i) >= txend) // constrain to tx buffer size
                                    break;

                                uint16_t res;
                                if(!MasterRegisters::getReg(ntoh16(query->start) + i, &res))
                                    break;

                                resp->values[i] = hton16(res);
                                resp->byteCount += sizeof(res);
                            }

                            // tx response
                            txheader->len = hton16(resp->byteCount + 2 + sizeof(*resp)); // +3 = [unit id] + [function] + [size of "byte count" variable]
                            uint16_t _len = sizeof(*txheader) + sizeof(*txadu) + sizeof(*resp) + resp->byteCount;
                            send(sn, txbuf, _len);

                        } break;

                    case Function::R_INPUT_REGS:
                        uart_ui.nputs(ARRANDN("R_INPUT_REGS unhandled" NEWLINE));
                        break;

                    case Function::W_COIL:
                        uart_ui.nputs(ARRANDN("W_COIL unhandled" NEWLINE));
                        break;

                    case Function::W_REG:
                        uart_ui.nputs(ARRANDN("W_REG unhandled" NEWLINE));
                        break;

                    case Function::W_COILS:
                        uart_ui.nputs(ARRANDN("W_COILS unhandled" NEWLINE));
                        break;

                    case Function::W_REGS:
                        uart_ui.nputs(ARRANDN("W_REGS unhandled" NEWLINE));
                        break;

                    default:
                        uart_ui.nputs(ARRANDN("Modbus: received unknown function : "));
                        uart_ui.put32d(rxadu->func);
                        uart_ui.nputs(ARRANDN(NEWLINE));
                        break;
                }

            } while(false);
        }

        // connection was lost / terminated
        // wait some time, maybe something went bonkers
        vTaskDelay(pdMS_TO_TICKS(1e3));
    }

    System::FailHard("ethModbus_task ended" NEWLINE);
    vTaskDelete(NULL);
}


/*** wizchip setup *******************************************/

void wiz_select(void)       { wiz_spi.takeResource(0); wiz_cs.set();  }
void wiz_deselect(void)     { wiz_cs.clear(); wiz_spi.giveResource(); }
uint8_t wiz_read_byte(void) {wiz_spi.takeResource(0);
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

    return 0 == wizchip_init(NULL,NULL);
}
