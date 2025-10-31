/*
 * BQ76952.hpp
 *
 *  Created on: Jul 20, 2025
 *      Author: FSAE
 */

#ifndef SRC_MIDDLEWARE_BQ76952_HPP_
#define SRC_MIDDLEWARE_BQ76952_HPP_

#include <stdint.h>

#include "Core/system.hpp"

#include "BQ769x2_PROTOCOL.hpp"


/*
 * a wrapper Class over the C style protocol library
 */
class BQ76952 {
public:
    System::I2C::I2C * i2c_controller;
    uint8_t i2c_addr;

//    void readAlarmStatus();
//    void readSafetyStatus();
//    void readPFStatus();

    /** reads a specific cells voltage
     * @param command : eg :  CmdDrt::Cell1Voltage, CmdDrt::StackVoltage, CmdDrt::LDPinVoltage, ...
     * @returns mV
     * - 16b resolution, units of 1mV
     * " –0.2 V to 5.5 V " - BQ769x2DS.10.1/35
     */
//    uint16_t readVoltage(BQ769X2_PROTOCOL::CmdDrt command);
//    void readSeriesCurrent();
//    float readTemperature(BQ769X2_PROTOCOL::CmdDrt command);
//    int16_t readCurrent();
//    void readPassQ();
//    void readFETStatus();
//    void readAllTemperatures();

    bool sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd cmd);

    bool sendSubcommandR(BQ769X2_PROTOCOL::Cmd cmd, uint8_t readOut[32]);
    bool sendSubcommandW(BQ769X2_PROTOCOL::Cmd cmd, uint8_t data);
    bool sendSubcommandW2(BQ769X2_PROTOCOL::Cmd cmd,uint16_t data);

    bool sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt command, uint16_t * readValue);
    bool sendDirectCommandW(BQ769X2_PROTOCOL::CmdDrt command, uint16_t data);

    bool setRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, uint32_t reg_data, uint8_t datalen);

    bool I2C_ReadReg(uint16_t reg_addr, void *reg_data, uint8_t count);
    bool I2C_WriteReg(uint16_t reg_addr, void *reg_data, uint8_t count);
    bool I2C_WriteReg(uint16_t reg_addr, uint8_t data);
};


#endif /* SRC_MIDDLEWARE_BQ76952_HPP_ */
