/*
 * bridge_CAN_Modbus.cpp
 *
 *  Created on: Dec 11, 2025
 *      Author: turtl
 */

#include "bridge_CAN_Modbus.hpp"
#include "Core/system.hpp"
#include "Core/common.h"
#include "Core/std alternatives/string.hpp"

bool Networking::Bridge::CANModbus::ModbusTCP_to_CAN(Modbus::MBAPHeader const * mbap, DL_MCAN_TxBufElement * tx) {
    using namespace CAN;
    using namespace Modbus;

    /*** input validation *****/

    // is user stupid
    if(mbap == NULL || tx == NULL)
        return false;

    // is Modbus
    if(ntoh16(mbap->protocolID) != MBAPHeader::PROTOCOL_ID_MODBUS)
        return false;

    // will translation fit in a single CAN-FD packet?
    if(ntoh16(mbap->len) > sizeof(CANPacket) - sizeof(CANPacket::_Header) + 1) // +1 for unitID which is part of the CANPacket
        return false;


    /*** translation **********/
    // - dont bother with ntoh and hton

    auto txid   = reinterpret_cast<J1939::ID *>(&tx->id);
    auto txpkt  = reinterpret_cast<CANPacket *>(tx->data);

    txid->pdu_format        = J1939_PDU_FORMAT;
    txid->pdu_specific      = mbap->adu[0].func;

    txpkt->header.transactionID = mbap->transactionID;
    txpkt->header.mbatlen   = ntoh16(mbap->len);
    txpkt->header.unitID    = mbap->unitID;

    static_assert(sizeof(txid->pdu_format) == sizeof(J1939_PDU_FORMAT));
    static_assert(sizeof(txid->pdu_specific) == sizeof(mbap->adu[0].func));
    static_assert(sizeof(txpkt->header.transactionID) == sizeof(mbap->transactionID));

    // copy ADU
    uint8_t aduDatalen = txpkt->header.mbatlen - sizeof(ADUPacket) - 1;//-1 for unitID
    ALT::memcpy(
            mbap->adu[0].data,
            txpkt->header.adudata,
            aduDatalen
        );


    /*** tx buffer settings ***/

    tx->dlc = System::CANFD::len2DLC(aduDatalen);
    tx->xtd = true; // use 29b CAN ID
    tx->fdf = true; // use CAN-FD format


    return true;
}

bool Networking::Bridge::CANModbus::CAN_to_ModbusTCP(DL_MCAN_RxBufElement const * rx, Modbus::MBAPHeader * mbap) {
    using namespace CAN;
    using namespace Modbus;

    auto rxid   = reinterpret_cast<J1939::ID const *>(&rx->id);
    auto rxpkt  = reinterpret_cast<CANPacket const *>(rx->data);

    /*** input validation *****/

    // is user stupid
    if(mbap == NULL || rx == NULL)
        return false;

    // is not 29b CAN ID
    if(!rx->xtd)
        return false;

    // is not CAN-FD
    if(!rx->fdf)
        return false;

    // is not PDU for a translated packet
    if(!(rxid->raw != J1939_PDU_FORMAT))
        return false;

    // is Modbus packet larger than CAN packet
    if(System::CANFD::DLC2Len(rx) < rxpkt->header.mbatlen + sizeof(CANPacket::_Header) - 1) // -1 for unitID
        return false;


    /*** translation **********/

    mbap->transactionID     = rxpkt->header.transactionID;
    mbap->protocolID        = MBAPHeader::PROTOCOL_ID_MODBUS;
    mbap->len               = ntoh16((uint16_t)rxpkt->header.mbatlen);
    mbap->unitID            = rxpkt->header.unitID;
    mbap->adu[0].func       = (Function)rxid->pdu_specific;

    // restore ADU
    uint8_t adulen = rxpkt->header.mbatlen - sizeof(ADUPacket) - sizeof(mbap->unitID);
    ALT::memcpy(
            rxpkt->header.adudata,
            mbap->adu[0].data,
            adulen
        );

    return true;
}


