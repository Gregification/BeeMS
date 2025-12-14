/*
 * Modbus.cpp
 *
 *  Created on: Dec 2, 2025
 *      Author: turtl
 */

#include "Modbus.hpp"
#include "Core/Networking/ModbusRegisters.hpp"
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
    if(rxlen < sizeof(MBAPHeader) || txlen < sizeof(MBAPHeader))
        return false;

    // is packet undersized according to Modbus
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

//                uart.nputs(ARRANDN());
//                uart.put32d();
                uart.nputs(ARRANDN("dump: "));
                for(uint8_t i = 0; i < sizeof(MBAPHeader) + ntoh16(rxheader->len); i++){
                    if(i % 10 == 0)
                        uart.nputs(ARRANDN(NEWLINE));

                    uart.putu32h(((uint8_t *)rxheader)[i]);
                    uart.nputs(ARRANDN(" "));
                }
                uart.nputs(ARRANDN(NEWLINE));
                uart.nputs(ARRANDN(NEWLINE "\t mbap.len: "));
                uart.put32d(ntoh16(rxheader->len));
                uart.nputs(ARRANDN(NEWLINE "\t mbap.protocolID: "));
                uart.put32d(ntoh16(rxheader->protocolID));
                uart.nputs(ARRANDN(NEWLINE "\t mbap.transactionID: "));
                uart.put32d(ntoh16(rxheader->transactionID));
                uart.nputs(ARRANDN(NEWLINE "\t mbap.adu[0].func: "));
                uart.put32d(rxheader->adu[0].func);
                uart.nputs(ARRANDN(NEWLINE "\t mbap.adu[0].unitID: "));
                uart.put32d(rxheader->adu[0].unitID);
                uart.nputs(ARRANDN(NEWLINE "\t RR.start: "));
                uart.put32d(ntoh16(query->start));
                uart.nputs(ARRANDN(NEWLINE "\t RR.len: "));
                uart.put32d(ntoh16(query->len));
                uart.nputs(ARRANDN(NEWLINE));

                /*** validation ***********/

                if(ntoh16(rxheader->len) < sizeof(ADUPacket) + sizeof(F_Range_REQ))
                    return false;

                if(txlen < sizeof(MBAPHeader) + sizeof(ADUPacket) + sizeof(F_Range_RES) + ntoh16(query->len) * sizeof(uint16_t)) { // constrain to tx buffer size
                    uart.nputs(ARRANDN("modbus response buffer undersized, needed : >" ));
                    uart.put32d(sizeof(MBAPHeader) + sizeof(ADUPacket) + sizeof(F_Range_RES) + ntoh16(query->len) * sizeof(uint16_t));
                    uart.nputs(ARRANDN(" \t, got: " ));
                    uart.put32d(txlen);
                    uart.nputs(ARRANDN(NEWLINE));
                    return false;
                }

                /*** response *************/

                *txheader   = *rxheader;
                txadu       = rxadu;
                resp->byteCount = sizeof(uint16_t) * ntoh16(query->len);
                txheader->len = hton16(resp->byteCount + sizeof(ADUPacket) + sizeof(F_Range_RES));

                for(uint16_t i = 0; i < ntoh16(query->len); i++){ // for each requested address
                    uint16_t res;

                    if(!VTRegisters::getReg(ntoh16(query->start) + i, &res))
                        return false;

                    uart.nputs(ARRANDN(" \t"));
                    uart.putu32h(res);
                    if(i % 10 == 0 && i != 0)
                        uart.nputs(ARRANDN(NEWLINE));

                    resp->val16[i] = hton16(res);
                }
                uart.nputs(ARRANDN(NEWLINE));

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
