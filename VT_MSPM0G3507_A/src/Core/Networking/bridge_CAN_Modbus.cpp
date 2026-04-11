/*
 * bridge_CAN_Modbus.cpp
 *
 *  Created on: Dec 11, 2025
 *      Author: turtl
 */
/**
 * - this packet mapping can be improved to get like 5B more room. some of the modbus data sent is redundant
 */


#include "bridge_CAN_Modbus.hpp"
#include "Core/system.hpp"
#include "Core/common.h"
#include "Core/std alternatives/string.hpp"
#include "BMSComms.hpp"

bool Networking::Bridge::CANModbus::ModbusTCP_to_CAN(Modbus::MBAPHeader const * mbap, DL_MCAN_TxBufElement * tx, Meta_t const * meta) {
    using namespace CAN;
    using namespace Modbus;

    /*** input validation *****/

    // is user stupid
    if(mbap == NULL || tx == NULL || meta == NULL)
        return false;

    // is Modbus
    if(ntoh16(mbap->protocolID) != MBAPHeader::PROTOCOL_ID_MODBUS)
        return false;

    // will translation fit in a single CAN-FD packet?
    uint8_t aduDataLen = ntoh16(mbap->len) - sizeof(ADUPacket);
    if(aduDataLen > sizeof(CANPacket) - sizeof(CANPacket::_Header))
        return false;

    /*** translation **********/
    // - dont bother with ntoh and hton
    // - THIS CODE MUST ABIDE BY BMSComms packet format

    *tx = {
            .id     = 0,        // CAN id, 11b->[28:18], 29b->[] when using 11b can id
            .rtr    = 0,        // 0: data frame, 1: remote frame
            .xtd    = 1,        // 0: 11b id, 1: 29b id
            .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
            .dlc    = 0,        // data byte count, see DL comments
            .brs    = 1,        // 0: no bit rate switching, 1: yes brs
            .fdf    = 1,        // FD format, 0: classic CAN, 1: CAN FD format
            .efc    = 0,        // 0: dont store Tx events, 1: store
            .mm     = 0,        // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
        };

    { // set up ID
        J1939::ID * id   = reinterpret_cast<J1939::ID *>(&(tx->id));
        id->src_addr    = BMSComms::getID();        // message from this device, used as modbus unit ID
        id->pdu_format  = BMSComms::J1939_PF_e::MOD;  // is a Modbus thing
        id->pdu_specific= meta->initiatorID;        // CAN packet goes to this device
        id->data_page   = 0;                        // unused
        id->priority    = BMSComms::PRI_MODBUS;     // standard priority

//        auto & uart = System::uart_ui;
//        uart.nputs(ARRANDN("TCP->CAN"));
//        uart.nputs(ARRANDN(NEWLINE   "\tsrc: \t" ));
//        uart.put32d(id->src_addr);
//        uart.nputs(ARRANDN(NEWLINE   "\tpf:  \t" ));
//        uart.put32d(id->pdu_format);
//        uart.nputs(ARRANDN(NEWLINE   "\tps:  \t" ));
//        uart.put32d(id->pdu_specific);
//        uart.nputs(ARRANDN(NEWLINE   "\tdp:  \t" ));
//        uart.put32d(id->data_page);
//        uart.nputs(ARRANDN(NEWLINE   "\tpri: \t" ));
//        uart.put32d(id->priority);
//        uart.nputs(ARRANDN(NEWLINE));
    }

    { // setup data
        CANPacket * pkt  = reinterpret_cast<CANPacket *>(tx->data);

        pkt->header.ipsocketnum     = meta->socket;
        pkt->header.transactionID   = mbap->transactionID;
        pkt->header.adu             = mbap->adu[0];
        pkt->header.aduDataLen      = aduDataLen;

        ALT::memcpy(
                mbap->adu[0].data,
                pkt->header.adu.data,
                aduDataLen
            );

        tx->dlc = System::CANFD::len2DLC(aduDataLen + sizeof(CANPacket::_Header));
    }

    /*** tx buffer settings ***/

    tx->xtd = true; // use 29b CAN ID
    tx->fdf = true; // use CAN-FD format

    return true;
}

bool Networking::Bridge::CANModbus::CAN_to_ModbusTCP(DL_MCAN_RxBufElement const * rx, Modbus::MBAPHeader * mbap, Meta_t * meta) {
    using namespace CAN;
    using namespace Modbus;

    auto rxid   = reinterpret_cast<J1939::ID const *>(&rx->id);
    auto rxpkt  = reinterpret_cast<CANPacket const *>(rx->data);

    /*** input validation *****/

    // is user stupid
    if(mbap == NULL || rx == NULL || meta == NULL)
        return false;

    // is not 29b CAN ID
    if(!rx->xtd)
        return false;

    // is not CAN-FD
    if(!rx->fdf)
        return false;

    // is not PDU for a translated packet
    if(rxid->pdu_format != BMSComms::J1939_PF_e::MOD)
        return false;

    // is socket funky
    if(rxpkt->header.ipsocketnum > 8) // w5500 limit
        return false;

    if(rxpkt->header.aduDataLen > sizeof(CANModbus::CANPacket) - sizeof(CANModbus::CANPacket::_Header))
        return false;

    /*** translation **********/

    { // assumed data
        mbap->protocolID        = hton16(MBAPHeader::PROTOCOL_ID_MODBUS);
    }

    { // strip info from data
        meta->socket            = rxpkt->header.ipsocketnum;
        mbap->transactionID     = rxpkt->header.transactionID;

        // restore ADU
        ALT::memcpy(
                rxpkt->header.adu.data,
                mbap->adu[0].data,
                rxpkt->header.aduDataLen
            );
    }

    { // strip info from ID
        mbap->len               = hton16(rxpkt->header.aduDataLen + sizeof(ADUPacket));
        mbap->adu[0]            = rxpkt->header.adu;

        meta->initiatorID       = rxid->src_addr;
    }

//    auto & uart = System::uart_ui;
//    auto & id = rxid;
//    uart.nputs(ARRANDN("CAN->TCP"));
//    uart.nputs(ARRANDN(NEWLINE   "\tsrc: \t" ));
//    uart.put32d(id->src_addr);
//    uart.nputs(ARRANDN(NEWLINE   "\tpf:  \t" ));
//    uart.put32d(id->pdu_format);
//    uart.nputs(ARRANDN(NEWLINE   "\tps:  \t" ));
//    uart.put32d(id->pdu_specific);
//    uart.nputs(ARRANDN(NEWLINE   "\tdp:  \t" ));
//    uart.put32d(id->data_page);
//    uart.nputs(ARRANDN(NEWLINE   "\tpri: \t" ));
//    uart.put32d(id->priority);
//    uart.nputs(ARRANDN(NEWLINE));

    return true;
}


