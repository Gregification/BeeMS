/*
 * BMSComms.cpp
 *
 *  Created on: Mar 16, 2026
 *      Author: turtl
 */

#include "BMSComms.hpp"
#include "Core/system.hpp"
#include "Core/MasterBoard.hpp"
#include "Core/std alternatives/string.hpp"

bool BMSComms::isValidPacketID(DL_MCAN_RxBufElement const & pkt) {
    if(pkt.xtd) {
        // is 29b id
        switch(System::CANFD::getID(pkt)) {
            default: break;

            case BMSComms::J1939_PF::MS:
            case BMSComms::J1939_PF::SM:
            case BMSComms::J1939_PF::MOD:
            case BMSComms::J1939_PF::B:
                return true;
        }
    } else {
        // is 11b id
        switch(System::CANFD::getID(pkt)) {
            default: break;
        }
    }

    return false;
}

bool BMSComms::sendPacket(J1939_PF type, uint8_t J1939_JS, int8_t priorityOffset, void const * data, uint8_t len) {
    if(len > MAX_PKT_SIZE_BYTES)
        return false;

    DL_MCAN_TxBufElement msg = {
            .id     = 0,        // CAN id, 11b->[28:18], 29b->[] when using 11b can id
            .rtr    = 0,        // 0: data frame, 1: remote frame
            .xtd    = 1,        // 0: 11b id, 1: 29b id
            .esi    = 0,        // error state indicator, 0: passive flag, 1: transmission recessive
            .dlc    = len,      // data byte count, see DL comments
            .brs    = 1,        // 0: no bit rate switching, 1: yes brs
            .fdf    = 1,        // FD format, 0: classic CAN, 1: CAN FD format
            .efc    = 0,        // 0: dont store Tx events, 1: store
            .mm     = 0,        // In order to track which transmit frame corresponds to which TX Event FIFO element, you can use the MM(Message Marker) bits in the transmit frame. The corresponding TX Event FIFO element will have the same message marker.
        };

    ALT::memcpy(data, msg.data, len);

    { // set up ID
        Networking::CAN::J1939::ID * id = reinterpret_cast<Networking::CAN::J1939::ID *>(&(msg.id));
        id->src_addr    = getID();
        id->pdu_format  = type;
        id->pdu_specific= J1939_JS;
        id->data_page   = 0;

        switch(type){
            case BMSComms::J1939_PF::MS:
            case BMSComms::J1939_PF::SM:
                id->priority = Networking::CAN::getPriOffset(priorityOffset, BASE_PRI_INT);
                break;

            case BMSComms::J1939_PF::MOD:
                id->priority = Networking::CAN::getPriOffset(priorityOffset, PRI_MODBUS);
                break;

            case BMSComms::J1939_PF::B:
            default:
                id->priority = BASE_PRI_BRD;
                break;
        }
    }

    { // transmit
        System::CANFD::canFD0.takeResource(pdMS_TO_TICKS(10));

        DL_MCAN_TxFIFOStatus tf;

        DL_MCAN_getTxFIFOQueStatus(System::CANFD::canFD0.reg, &tf);

        if(tf.freeLvl == 0) {
            System::CANFD::canFD0.giveResource();
            return false;
        }

        DL_MCAN_writeMsgRam(System::CANFD::canFD0.reg, DL_MCAN_MEM_TYPE_FIFO, tf.putIdx, &msg);
        DL_MCAN_TXBufAddReq(System::CANFD::canFD0.reg, tf.getIdx);

        System::CANFD::canFD0.giveResource();
    }

    return true;
}

uint8_t BMSComms::getID() {
    return MstrB::getUnitBoardID();
}

static_assert(BMSComms::BASE_PRI_ERROR <= 0b111, "limit of J1939 pkt");
static_assert(BMSComms::BASE_PRI_INT <= 0b111, "limit of J1939 pkt");
static_assert(BMSComms::BASE_PRI_BRD <= 0b111, "limit of J1939 pkt");
static_assert(BMSComms::PRI_MODBUS <= 0b111, "limit of J1939 pkt");
