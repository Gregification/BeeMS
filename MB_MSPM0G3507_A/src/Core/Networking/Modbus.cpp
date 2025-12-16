/*
 * Modbus.cpp
 *
 *  Created on: Dec 2, 2025
 *      Author: turtl
 */

#include <Core/Networking/ModbusRegisters.hpp>
#include "Modbus.hpp"
#include "Core/system.hpp"


bool Networking::Modbus::ProcessRequest(MBAPHeader const * rxheader, buffersize_t rxlen, MBAPHeader * txheader, buffersize_t txlen) {
    auto & uart = System::uart_ui;
    auto & rxadu = rxheader->adu[0];
    auto & txadu = txheader->adu[0];


    /*** general validation ***********************************/

    // is user bonkers
    if(!rxheader || !txheader)
        return false;

    // is user stupid
    if(rxlen < sizeof(MBAPHeader) + ntoh16(rxheader->len))
        return false;


    /*** packet specific response *****************************/

    switch (rxadu.func) {
        case Function::R_COILS:
            uart.nputs(ARRANDN("R_COILS unhandled" NEWLINE));
            return false;
            break;

        case Function::R_DISRETE_INPUTS:
            uart.nputs(ARRANDN("R_DISRETE_INPUTS unhandled" NEWLINE));
            return false;
            break;

        case Function::R_INPUT_REGS:
        case Function::R_HOLDING_REGS: {
                auto query  = reinterpret_cast<F_Range_REQ const *>(rxadu.data);
                auto resp   = reinterpret_cast<F_Range_RES *>(txadu.data);

                /*** validation ***********/

                if(ntoh16(rxheader->len) < sizeof(ADUPacket) + sizeof(F_Range_REQ))
                    return false;


                /*** response *************/

                *txheader   = *rxheader;
                txadu       = rxadu;
                resp->byteCount = 0;

                for(uint16_t i = 0; i < ntoh16(query->len); i++){ // for each requested address
                    uint16_t res;

                    if(txlen < sizeof(MBAPHeader) + sizeof(ADUPacket) + sizeof(res) + sizeof(res) * i) // constrain to tx buffer size
                        return false;

                    if(!MasterRegisters::getReg(ntoh16(query->start) + i, &res))
                        return false;

                    resp->val16[i] = hton16(res);
                    resp->byteCount += sizeof(res);
                }

                txheader->len = hton16(resp->byteCount + sizeof(F_Range_RES) + sizeof(ADUPacket));

                return true;
            } break;

        case Function::W_COIL:
            uart.nputs(ARRANDN("W_COIL unhandled" NEWLINE));
            return false;
            break;

        case Function::W_REG:
            uart.nputs(ARRANDN("W_REG unhandled" NEWLINE));
            return false;
            break;

        case Function::W_COILS:
            uart.nputs(ARRANDN("W_COILS unhandled" NEWLINE));
            return false;
            break;

        case Function::W_REGS:
            uart.nputs(ARRANDN("W_REGS unhandled" NEWLINE));
            return false;
            break;

        default:
            uart.nputs(ARRANDN("Modbus: received unknown function : "));
            uart.put32d(rxadu.func);
            uart.nputs(ARRANDN(NEWLINE));
            return false;
            break;
    }

}
