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

#include <Core/CETRX.hpp>
#include <cstdint>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>

#include <Tasks/task_CAN_Eth_TRX.hpp>
#include "Core/system.hpp"
#include "Core/std alternatives/string.hpp"
#include "Middleware/W5500/socket.h"
#include "Middleware/W5500/wizchip_conf.h"


/*** setup ***************************************************/
System::UART::UART &            uart = System::UART::uart_ui;

/*** CAN ***************/
DL_MCAN_RX_FIFO_NUM constexpr   canfifo = DL_MCAN_RX_FIFO_NUM::DL_MCAN_RX_FIFO_NUM_0;


/*** wizchip setup ****/
System::SPI::SPI &              wiz_spi   = CEB::Bridge::wiz_spi;
System::GPIO::GPIO const &      wiz_cs    = CEB::Bridge::wiz_cs;
System::GPIO::GPIO const &      wiz_reset = CEB::Bridge::wiz_reset;
//System::SPI::SPI &              wiz_spi   = System::SPI::spi1;
//System::GPIO::GPIO const &      wiz_cs    = System::GPIO::PB3;
//System::GPIO::GPIO const &      wiz_reset = System::GPIO::PB2;
//System::SPI::SPI &              wiz_spi   = System::SPI::spi0; // workie
//System::GPIO::GPIO const &      wiz_cs    = System::GPIO::PA15;
//System::GPIO::GPIO const &      wiz_reset = System::GPIO::PB14;

/*************************************************************/

union _RXBuffer {
    DL_MCAN_RxBufElement canrx;

    uint8_t arr[500]; // for ethernet
};
static_assert(sizeof(_RXBuffer) <= UINT16_MAX);

union _TXBuffer {
    DL_MCAN_TxBufElement cantx;
    DL_MCAN_RxBufElement canrx;

    uint8_t arr[500]; // for ethernet
};
static_assert(sizeof(_TXBuffer) <= UINT16_MAX);


/** returns true on success */
bool setupWizchip();
/** prints human readable meaning of socket error code */
void wiz_print_sockerror(int8_t error);

uint8_t Can2Eth(_RXBuffer const * rx, _TXBuffer * tx);
uint8_t Eth2Can(_RXBuffer const * rx, _TXBuffer * tx);

bool canToEthHandler(SOCKET sn, _RXBuffer * rxb, _TXBuffer * txb);

/*** main ****************************************************/

void Task::ethcan_task(void *){
    uart.nputs(ARRANDN("ethModbus_task start" NEWLINE));

    {
        CEB::Bridge::netConfig.mac[5] = CEB::getUnitBoardID();
        CEB::Bridge::netConfig.mac[4] = System::mcuID;
        CEB::Bridge::netConfig.mac[3] = System::mcuID >> 8;
        CEB::Bridge::netConfig.ip[3]  = System::mcuID;
    }

    _RXBuffer rxbuf = {0};
    _TXBuffer txbuf = {0};


    wiz_spi.setSCLKTarget(10e6);


    /*** W5500 init *****************/

    uart.nputs(ARRANDN(CLIHIGHLIGHT "awaiting W5500 response ..." CLIRESET NEWLINE));
    while(!setupWizchip()) {
        uart.nputs(ARRANDN(CLIERROR "non-fatal: W5500 init failed! retrying..." CLIRESET NEWLINE));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    wizchip_setnetinfo(&CEB::Bridge::netConfig);

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
        uart.putu32d(CEB::Bridge::netConfig.ip[i]);
        uart.nputs(ARRANDN("."));
    }
    uart.nputs(ARRANDN(NEWLINE "SNM: \t"));
    for(int i = 0; i < 4; i++){
        uart.putu32d(CEB::Bridge::netConfig.sn[i]);
        uart.nputs(ARRANDN("."));
    }
    uart.nputs(ARRANDN(NEWLINE "MAC: \t"));
    for(int i = 0; i < 6; i++){
        uart.putu32h(CEB::Bridge::netConfig.mac[i]);
        uart.nputs(ARRANDN(" "));
    }
    uart.nputs(ARRANDN(CLIRESET NEWLINE));

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

    int32_t error;
    SOCKET sn;
    bool reset;

    while(true){
        close(sn);
        reset = false;
        sn = CEB::Bridge::wiz_sn;

        /*** init socket ****************/

        switch(error = socket(sn, Sn_MR_UDP, CEB::Bridge::wiz_IP_port, SF_IO_NONBLOCK)){
            default:
                if(error == sn)
                    // yippie
                    break;

                uart.nputs(ARRANDN("W5500 unknown response to socket!" NEWLINE));

            case SOCKERR_SOCKMODE:
            case SOCKERR_SOCKFLAG:
            case SOCKERR_SOCKNUM:
                // failed to init socket, retry
                uart.nputs(ARRANDN(CLIERROR "CanEthBridge: failed to init socket : " ));
                wiz_print_sockerror(error);
                uart.nputs(ARRANDN(CLIRESET NEWLINE));
                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
        }

        /*** wait for incoming connection ***/

        uart.nputs(ARRANDN("CanEthBridge: listening for UDP (port: " ));
        uart.putu32d(CEB::Bridge::wiz_IP_port);
        uart.nputs(ARRANDN("), and CAN bus"));
        uart.nputs(ARRANDN(CLIRESET NEWLINE));

        // wait for eth packet or can packet, & respond.
        while(!reset) {

            switch(error = recvfrom(
                    sn,
                    rxbuf.arr,
                    sizeof(rxbuf.arr),
                    CEB::Bridge::ethBroadcastIP,
                    &CEB::Bridge::wiz_IP_port)
            ) {
                default:
                    if(error > 0){
                        System::UART::uart_ui.nputs(ARRANDN("eth rx len: "));
                        System::UART::uart_ui.putu32d(error);
                        System::UART::uart_ui.nputs(ARRANDN("" NEWLINE));
                        break;
                    }
                case SOCKERR_SOCKMODE:
                case SOCKERR_SOCKNUM:
                    System::UART::uart_ui.nputs(ARRANDN(CLIERROR "socket error: "));
                    System::UART::uart_ui.putu32d(error);
                    System::UART::uart_ui.nputs(ARRANDN("; restarting..." CLIRESET NEWLINE));

                    // restart
                    reset = true;
                    break;

                case SOCK_BUSY:
                case SOCKERR_DATALEN:
                    /*** can 2 eth ******************/
                    if(!canToEthHandler(sn, &rxbuf, &txbuf)) {
                        // keep waiting
//                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                    continue;
            }

            if(reset) break;

            /*** eth 2 can ******************/

            if(error != sizeof(rxbuf.canrx))
                continue;

            if(Eth2Can(&rxbuf, &txbuf)) {
                // tx can

                DL_MCAN_TxFIFOStatus tf;
                DL_MCAN_getTxFIFOQueStatus(CEB::Bridge::can.reg, &tf);

                if(tf.fifoFull){
//                    System::UART::uart_ui.nputs(ARRANDN("dropped, CAN TX FIFO full" NEWLINE));
                    continue;
                }
//                System::UART::uart_ui.nputs(ARRANDN("TX from buffer "));
//                System::UART::uart_ui.putu32d(tf.putIdx);
//                System::UART::uart_ui.nputs(ARRANDN("" NEWLINE));

                DL_MCAN_writeMsgRam(CEB::Bridge::can.reg, DL_MCAN_MEM_TYPE_BUF, tf.putIdx, &txbuf.cantx);
                DL_MCAN_TXBufAddReq(CEB::Bridge::can.reg, tf.putIdx);

                DL_GPIO_togglePins(GPIOPINPUX(CEB::Indi::LED::ethRX));

//                System::UART::uart_ui.nputs(ARRANDN(CLIYES "eth -> can done" CLIRESET NEWLINE));
            }
        }

    }


    System::FailHard("ethModbus_task ended" NEWLINE);
    vTaskDelete(NULL);
}

bool canToEthHandler(SOCKET sn, _RXBuffer * rxb, _TXBuffer * txb) {
    using namespace CEB::Bridge;

    DL_MCAN_RxFIFOStatus rf;
    rf.num = DL_MCAN_RX_FIFO_NUM_0;
    DL_MCAN_getRxFIFOStatus(can.reg, &rf);

    if(rf.fillLvl == 0) // if CAN rx buffer is empty
        return false;

    DL_MCAN_readMsgRam(can.reg, DL_MCAN_MEM_TYPE_FIFO, 0, rf.num, &rxb->canrx);
    DL_MCAN_writeRxFIFOAck(can.reg, rf.num, rf.getIdx);

//    System::UART::uart_ui.nputs(ARRANDN("rx CAN ID: 0x"));
//    System::UART::uart_ui.putu32h(System::CANFD::getCANID(&rxb->canrx));

    uint8_t len = Can2Eth(rxb, txb);

    if(len == 0) {
//        System::UART::uart_ui.nputs(ARRANDN("\t -> \t ignored" NEWLINE));
        return false;
    }

//    System::UART::uart_ui.nputs(ARRANDN("\t -> \t forwarded to Eth" NEWLINE));

    sendto(sn, txb->arr, len, CEB::Bridge::ethBroadcastIP, CEB::Bridge::wiz_IP_port);

    DL_GPIO_togglePins(GPIOPINPUX(CEB::Indi::LED::canRX));

    return true;
}


uint8_t Can2Eth(_RXBuffer const * rx, _TXBuffer * tx) {
    tx->canrx = rx->canrx;
    tx->canrx.anmf = 0xBEEF;
    tx->canrx.fidx = System::mcuID;
    return sizeof(tx->canrx);
}

uint8_t Eth2Can(_RXBuffer const * rx, _TXBuffer * tx){
    if(     rx->canrx.anmf != 0xBEEF
        ||  rx->canrx.fidx == System::mcuID
        ) {
//        System::UART::uart_ui.nputs(ARRANDN("dropped for meow" NEWLINE));
        return false;
    }

    ALT::memcpy(rx->canrx.data, tx->cantx.data, sizeof(tx->cantx.data));
    tx->cantx.id = rx->canrx.id;
    tx->cantx.rtr = rx->canrx.rtr;
    tx->cantx.xtd = rx->canrx.xtd;
    tx->cantx.dlc = rx->canrx.dlc;
    tx->cantx.brs = rx->canrx.brs;
    tx->cantx.fdf = rx->canrx.fdf;
    tx->cantx.efc = 0;
    return sizeof(tx->cantx);
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
