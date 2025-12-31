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

#include <Core/Networking/ModbusRegisters.hpp>
#include <cstdint>
#include <FreeRTOS.h>
#include <task.h>
#include <ti/driverlib/driverlib.h>
#include <Tasks/task_ethModbus.hpp>

#include "Core/system.hpp"
#include "Core/MasterBoard.hpp"
#include "Core/std alternatives/string.hpp"
#include "Core/Networking/Modbus.hpp"
#include "Core/Networking/bridge_CAN_Modbus.hpp"
#include "Middleware/W5500/socket.h"
#include "Middleware/W5500/wizchip_conf.h"


/*** setup ***************************************************/
System::UART::UART &            uart = System::UART::uart_ui;

/*** CAN ***************/
System::CANFD::CANFD &          can = System::CANFD::canFD0;
DL_MCAN_RX_FIFO_NUM constexpr   canfifo = DL_MCAN_RX_FIFO_NUM::DL_MCAN_RX_FIFO_NUM_0;


/*** wizchip setup ****/
System::SPI::SPI &              wiz_spi   = MstrB::Eth::spi;
System::GPIO::GPIO const &      wiz_cs    = MstrB::Eth::cs;
System::GPIO::GPIO const &      wiz_reset = MstrB::Eth::reset;
uint16_t const                  modbusTCPPort = 502;   // arbitrary, must match RapidSCADA connection settings
uint8_t const                   socketNum = 0;          // [0,8], arbitrary, socket must not be used elsewhere

wiz_NetInfo netConfig = {
           .mac = {0xBE,0xEE,0xEE,0x00,0x00,0x00}, // arbitrary
           .ip  = {192,168,1,252},
           .sn  = {255,255,255,0},
           .gw  = {192,168,1,1},
           .dns = {8,8,8,8},
           .dhcp= NETINFO_STATIC
    };


/*************************************************************/

union _RXBuffer {
    Networking::Modbus::MBAPHeader mbap;
    DL_MCAN_RxBufElement canrx;

    uint8_t arr[Networking::Bridge::CANModbus::PKTBUFFSIZE]; // 260B MAX standard Modbus-TCP len (MBAP + PDU), we can get away with smaller for 99% of stuff
};
static_assert(sizeof(_RXBuffer) <= UINT16_MAX);

union _TXBuffer {
    Networking::Modbus::MBAPHeader mbap;
    DL_MCAN_TxBufElement cantx;

    uint8_t arr[Networking::Bridge::CANModbus::PKTBUFFSIZE]; // see comment above. idk. what do u want from me. go find the campus cat. ("mmmm" (Microwave imitation))
};
static_assert(sizeof(_TXBuffer) <= UINT16_MAX);


/** returns true on success */
bool setupWizchip();
/** prints human readable meaning of socket error code */
void wiz_print_sockerror(int8_t error);

bool forwardModbusTCP2CAN(_RXBuffer const * rxbuff, _TXBuffer * txbuff);


/*** main ****************************************************/

void Task::ethModbus_task(void *){
    uart.nputs(ARRANDN("ethModbus_task start" NEWLINE));
    _RXBuffer rxbuf = {0};
    _TXBuffer txbuf = {0};

    wiz_spi.setSCLKTarget(2e6);


    /*** W5500 init *****************/

    uart.nputs(ARRANDN(CLIHIGHLIGHT "awaiting W5500 response ..." CLIRESET NEWLINE));
    while(!setupWizchip()) {
        uart.nputs(ARRANDN(CLIERROR "non-fatal: W5500 init failed!" CLIRESET NEWLINE));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    for(int i = 0; i < 100; i++){
        DL_GPIO_togglePins(GPIOPINPUX(MstrB::Indi::i1));
        vTaskDelay(pdMS_TO_TICKS(200));
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
     * 5. process or forward request
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

        while(!reset){
            // make life simple
            using namespace Networking::CAN;
            using namespace Networking::Modbus;
            using namespace Networking::Bridge::CANModbus;

            // CAN -> ModbusTCP
            {
                DL_MCAN_RxFIFOStatus rf = {.num = canfifo};
                DL_MCAN_getRxFIFOStatus(can.reg, &rf);

                for(uint32_t i = rf.fillLvl; i != 0; i--) {
                    if(rf.fillLvl == 0)
                        break;

                    DL_MCAN_readMsgRam(can.reg, DL_MCAN_MEM_TYPE_FIFO, 0, rf.num, &rxbuf.canrx);
                    DL_MCAN_writeRxFIFOAck(can.reg, rf.num, rf.getIdx);

                    /*** forward to ModbusTCP *********/

                    uart.nputs(ARRANDN("CAN rx: " ));
                    uart.nputs(ARRANDN(NEWLINE "dump: "));
                    for(uint8_t j = 0; j < System::CANFD::DLC2Len(&rxbuf.canrx); j++){
                        if(j % 10 == 0)
                            uart.nputs(ARRANDN(NEWLINE));

                        uart.nputs(ARRANDN(" "));
                        uart.putu32h(rxbuf.canrx.data[j]);
                    }
                    uart.nputs(ARRANDN(NEWLINE));

                    if(CAN_to_ModbusTCP(&rxbuf.canrx, &txbuf.mbap)) {
                        send(sn, txbuf.arr, sizeof(MBAPHeader) + ntoh16(txbuf.mbap.len));
                        uart.nputs(ARRANDN("forwarded to ModbusTCP"));
                    } else {
                        uart.nputs(ARRANDN("not forwarded to ModbusTCP"));
                    }
                    uart.nputs(ARRANDN(NEWLINE));

                    // already bad
//                    uart.nputs(ARRANDN("dump: "));
//                    for(uint8_t j = 0; j < sizeof(MBAPHeader) + ntoh16(txbuf.mbap.len); j++){
//                        if(j % 10 == 0)
//                            uart.nputs(ARRANDN(NEWLINE));
//
//                        uart.nputs(ARRANDN(" "));
//                        uart.putu32h(txbuf.arr[j]);
//                    }
//                    uart.nputs(ARRANDN(NEWLINE));

                    DL_MCAN_getRxFIFOStatus(can.reg, &rf);
                }
            }

            // rx ModbusTCP packet
            int32_t status = recv(sn, ARRANDN(rxbuf.arr));
            switch(status){
                static uint8_t cycles = 1; // 'exponential'(allegedly) back off wait times

                case SOCK_BUSY:
                    if(cycles < 15) // back off max cap mS
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
            void const * const rxend = (void*)(rxbuf.arr + rxlen);

            do {

                MBAPHeader const * rxheader = &rxbuf.mbap;
                ADUPacket const * rxadu     = rxheader->adu;
                MBAPHeader * txheader       = &txbuf.mbap;
                ADUPacket * txadu           = txheader->adu;


                /*** validation *****************/

                if(rxadu >= rxend) // is under sized?
                    break; // ignore packet

                if(ntoh16(rxheader->protocolID) != MBAPHeader::PROTOCOL_ID_MODBUS) // is protocol not Modbus ?
                    break; // ignore packet


                /*** process packet *************/

                uart.nputs(ARRANDN("TCP rx: unitID: "));
                uart.putu32h(rxheader->adu[0].unitID);
                uart.nputs(ARRANDN(NEWLINE));

                // is intended for this device?
                if(rxheader->adu[0].unitID != MstrB::getUnitBoardID()) {
                    //  forward packet over CAN to intended device
                    uart.nputs(ARRANDN("\t forwarded to CAN bus." NEWLINE));
                    if(forwardModbusTCP2CAN(&rxbuf, &txbuf)) {
                        uart.nputs(ARRANDN("\tforward: success" NEWLINE));
                    } else {
                        uart.nputs(ARRANDN("\tforward: fail" NEWLINE));
                    }
                    continue;
                }

                if(ProcessRequest(rxheader, sizeof(rxbuf.arr), txheader, sizeof(txbuf.arr)))
                    send(sn, txbuf.arr, sizeof(MBAPHeader) + ntoh16(txbuf.mbap.len));

            } while(false);
        }

        // connection was lost / terminated
        // wait some time, maybe something went bonkers
        vTaskDelay(pdMS_TO_TICKS(1e3));
    }

    System::FailHard("ethModbus_task ended" NEWLINE);
    vTaskDelete(NULL);
}

bool forwardModbusTCP2CAN(_RXBuffer const * rx, _TXBuffer * buf) {
    // NOTE: resource LOCKS CAN

    using namespace Networking::Bridge::CANModbus;
    using namespace Networking::Modbus;
    using namespace Networking::CAN;

    MBAPHeader const * rxheader = reinterpret_cast<MBAPHeader const *>(rx->arr);

    /*** forward ModbusTCP to CAN *************/

    {
        auto id = reinterpret_cast<J1939::ID *>(&buf->cantx.id);

        buf->cantx = {
                .id     = 0,        // CAN id, 11b->[28:18], 29b->[] when using 11b can id
                .rtr    = 0,        // 0: data frame, 1: remote frame
                .xtd    = 1,        // 0: 11b id, 1: 29b id
                .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
                .dlc    = 0,        // data byte count, see DL comments
                .brs    = 1,        // 0: no bit rate switching, 1: yes brs
                .fdf    = 1,        // FD format, 0: classic CAN, 1: CAN FD format
                .efc    = 0,        // 0: dont store Tx events, 1: store
                .mm     = rxheader->transactionID,  // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
            };

        if(!ModbusTCP_to_CAN(rxheader, &(buf->cantx))) {
            uart.nputs(ARRANDN("failed ModbusTCP_to_CAN" NEWLINE));
            return false;
        }

        id->src_addr = MstrB::getUnitBoardID();
        id->priority = 0b110;

        // send CAN packet
        if(!can.takeResource(pdMS_TO_TICKS(50))) { // LOCK CAN
            uart.nputs(ARRANDN("CAN lock failed" NEWLINE));
            return false;
        }

        DL_MCAN_TxFIFOStatus tf;

        // wait for fifo space
        uint8_t i = 4;
        for(; i > 0; --i) {
            DL_MCAN_getTxFIFOQueStatus(can.reg, &tf);

            if(tf.fifoFull) {
                vTaskDelay(pdMS_TO_TICKS(5));
                continue;
            }

//            uint32_t bufferIndex = tf.putIdx;
//            uart.nputs(ARRANDN("TX from buffer "));
//            uart.putu32d(bufferIndex);
//            uart.nputs(ARRANDN("" NEWLINE));

            DL_MCAN_writeMsgRam(can.reg, DL_MCAN_MEM_TYPE_FIFO, tf.putIdx, &buf->cantx);
            DL_MCAN_TXBufAddReq(can.reg, tf.getIdx);
            break;
        }

        can.giveResource(); // UNLOCK CAN

        if(i == 0) {
//            uart.nputs(ARRANDN("CAN tx fifo timeout" NEWLINE));
            return false;
        }

    }


    /*** forward CAN response to Modbus TCP ***/
    // no need. all CAN-Modbus packets will be automatically forwarded

    return true;
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
