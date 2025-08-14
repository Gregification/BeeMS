/*
 * BQ76952.hpp
 *
 *  Created on: Jul 20, 2025
 *      Author: FSAE
 */

#ifndef SRC_MIDDLEWARE_BQ76952_HPP_
#define SRC_MIDDLEWARE_BQ76952_HPP_

#include "Core/system.hpp"

#include <stdint.h>

/*
 * a wrapper Class over the C style protocol library
 */
class BQ76952 {
public:
    static constexpr TickType_t I2C_MIN_TIMEOUT = pdMS_TO_TICKS(4);

    System::I2C::I2C * i2c_controller;
    uint8_t i2c_addr;

    void init();
    void readAlarmStatus();
    void readSafetyStatus();
    void readPFStatus();

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

    bool I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout);
    bool I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout);
};


#endif /* SRC_MIDDLEWARE_BQ76952_HPP_ */
