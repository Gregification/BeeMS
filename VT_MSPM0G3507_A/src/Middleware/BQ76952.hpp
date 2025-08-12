/*
 * BQ76952.hpp
 *
 *  Created on: Jul 20, 2025
 *      Author: FSAE
 */

#ifndef SRC_MIDDLEWARE_BQ76952_HPP_
#define SRC_MIDDLEWARE_BQ76952_HPP_

#include "Core/system.hpp"
#include "Middleware/BQ769x2_PROTOCOL.hpp"

class BQ76952 {
using namespace BQ769X2_PROTOCOL;
public:
    System::I2C::I2C i2cBus;
    BQ769X2_PROTOCOL::I2C_addr addr;


    void init(tGaug5eApplication *pGaugeApp);
    void readAlarmStatus();
    void readSafetyStatus();
    void readPFStatus();

    /** reads a specific cells voltage
     * @param command : eg :  CmdDrt::Cell1Voltage, CmdDrt::StackVoltage, CmdDrt::LDPinVoltage, ...
     * @returns mV
     * - 16b resolution, units of 1mV
     * " –0.2 V to 5.5 V " - BQ769x2DS.10.1/35
     */
    uint16_t readVoltage(CmdDrt command);
    void readSeriesCurrent();
    float readTemperature(CmdDrt command);
    void readCurrent();
    void readPassQ();
    void readFETStatus();
    void readAllTemperatures();
};


#endif /* SRC_MIDDLEWARE_BQ76952_HPP_ */
