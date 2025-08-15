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
    static constexpr TickType_t I2C_MIN_TIMEOUT = pdMS_TO_TICKS(4);

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

    bool sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd cmd, TickType_t timeout = I2C_MIN_TIMEOUT);

    bool sendSubcommandR(BQ769X2_PROTOCOL::Cmd cmd, uint8_t readOut[32], TickType_t timeout = I2C_MIN_TIMEOUT);
    bool sendSubcommandW(BQ769X2_PROTOCOL::Cmd cmd, uint8_t data,        TickType_t timeout = I2C_MIN_TIMEOUT);
    bool sendSubcommandW2(BQ769X2_PROTOCOL::Cmd cmd,uint16_t data,       TickType_t timeout = I2C_MIN_TIMEOUT);

    bool sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt command, uint16_t * readValue, TickType_t timeout = I2C_MIN_TIMEOUT);
    bool sendDirectCommandW(BQ769X2_PROTOCOL::CmdDrt command, uint16_t data, TickType_t timeout = I2C_MIN_TIMEOUT);

    bool setRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, uint32_t reg_data, uint8_t datalen, TickType_t timeout = I2C_MIN_TIMEOUT);

    bool I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout = I2C_MIN_TIMEOUT);
    bool I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout = I2C_MIN_TIMEOUT);
};


#endif /* SRC_MIDDLEWARE_BQ76952_HPP_ */
