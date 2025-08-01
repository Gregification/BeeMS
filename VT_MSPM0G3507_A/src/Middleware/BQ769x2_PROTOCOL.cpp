/*
 * Copyright (c) 2024, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <FreeRTOS.h>
#include <task.h>

#include "Middleware/BQ769x2_PROTOCOL.hpp"

//******************************************************************************
// BQ Parameters ***************************************************************
//******************************************************************************
// Global Variables for cell voltages, temperatures, Stack voltage, PACK Pin voltage, LD Pin voltage, CC2 current
uint16_t CellVoltage[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
float Temperature[3]     = {0, 0, 0};
uint16_t Stack_Voltage   = 0x00;
uint16_t Pack_Voltage    = 0x00;
uint16_t LD_Voltage      = 0x00;
int16_t Pack_Current     = 0x00;
uint16_t AlarmBits       = 0x00;

uint8_t value_SafetyStatusA;  // Safety Status Register A
uint8_t value_SafetyStatusB;  // Safety Status Register B
uint8_t value_SafetyStatusC;  // Safety Status Register C
uint8_t value_PFStatusA;      // Permanent Fail Status Register A
uint8_t value_PFStatusB;      // Permanent Fail Status Register B
uint8_t value_PFStatusC;      // Permanent Fail Status Register C
uint8_t FET_Status;  // FET Status register contents  - Shows states of FETs
uint16_t CB_ActiveCells;  // Cell Balancing Active Cells

uint8_t UV_Fault             = 0;  // under-voltage fault state
uint8_t OV_Fault             = 0;  // over-voltage fault state
uint8_t SCD_Fault            = 0;  // short-circuit fault state
uint8_t OCD_Fault            = 0;  // over-current fault state
uint8_t ProtectionsTriggered = 0;  // Set to 1 if any protection triggers

uint8_t LD_ON = 0;  // Load Detect status bit
uint8_t DSG   = 0;  // discharge FET state
uint8_t CHG   = 0;  // charge FET state
uint8_t PCHG  = 0;  // pre-charge FET state
uint8_t PDSG  = 0;  // pre-discharge FET state

uint32_t AccumulatedCharge_Int;   // in AFE_READPASSQ func
uint32_t AccumulatedCharge_Frac;  // in AFE_READPASSQ func
uint32_t AccumulatedCharge_Time;  // in AFE_READPASSQ func

uint8_t RX_32Byte[32] = {0x00};
uint8_t RX_data[2]    = {0x00};

//**********************************BQ Parameters *********************************


//**********************************Function prototypes **********************************
void delayUS(uint16_t us)
// Sets the delay in microseconds.
{
//    uint16_t ms;
//    char i;
//    ms = us / 1000;
//    for (i = 0; i < ms; i++) delay_cycles(32000);
    vTaskDelay(pdMS_TO_TICKS(us / 1000) + 1);
}

uint8_t Checksum(uint8_t *ptr, uint8_t len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.
{
    uint8_t i;
    uint8_t checksum = 0;

    for (i = 0; i < len; i++) checksum += ptr[i];

    checksum = 0xff & ~checksum;

    return (checksum);
}

uint8_t CRC8(uint8_t *ptr, uint8_t len)
//Calculates CRC8 for passed bytes. Used in i2c read and write functions
{
    uint8_t i;
    uint8_t crc = 0;
    while (len-- != 0) {
        for (i = 0x80; i != 0; i /= 2) {
            if ((crc & 0x80) != 0) {
                crc *= 2;
                crc ^= 0x107;
            } else
                crc *= 2;

            if ((*ptr & i) != 0) crc ^= 0x107;
        }
        ptr++;
    }
    return (crc);
}

void BQ769X2_PROTOCOL::sendDirectCommand(CmdDrt command, uint16_t data, DIR_CMD_TYPE type)
// See the TRM or the BQ76952 header file for a full list of Direct Commands.
// For read type, user can read data from command address and stored in the Rx state of global variable to be read.
// For write type, user can write the data to the command address.
{  //type: R = read, W = write
    uint8_t TX_data[2] = {0x00, 0x00};

    //little endian format
    TX_data[0] = data & 0xff;
    TX_data[1] = (data >> 8) & 0xff;

    if (type == R) {                       //Read
        I2C_ReadReg(command, RX_data, 2);  //RX_data is a global variable
        delayUS(2000);
        delayUS(2000);  //success in 100k
    }
    if (type == W) {  //write
        //Control_status, alarm_status, alarm_enable all 2 bytes long
        I2C_WriteReg(command, TX_data, 2);
        delayUS(2000);
        delayUS(2000);
    }
}

void BQ769X2_PROTOCOL::sendCommandSubcommand(Cmd command)  //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
// All that this function do is formatting the transfer array then writing the array to hex 3E,
// the monitor will then operate based on the command.
{  //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

    uint8_t TX_Reg[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    I2C_WriteReg(0x3E, TX_Reg, 2);
    delayUS(2000);
}

void BQ769X2_PROTOCOL::sendSubcommand(Cmd command, uint16_t data, DIR_CMD_TYPE type)
// See the TRM or the BQ76952 header file for a full list of Subcommands
// The input type can either be the defined macros R for read, W for write, or W2 for write two bytes.
{
    //security keys and Manu_data writes dont work with this function (reading these commands works)
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    uint8_t TX_Reg[4]    = {0x00, 0x00, 0x00, 0x00};
    uint8_t TX_Buffer[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    if (type == BQ769X2_PROTOCOL::DIR_CMD_TYPE::R) {  //read
        I2C_WriteReg(0x3E, TX_Reg, 2);
        delayUS(2000);
        I2C_ReadReg(0x40, RX_32Byte, 32);  //RX_32Byte is a global variable
    } else if (type == BQ769X2_PROTOCOL::DIR_CMD_TYPE::W) {
        //FET_Control, REG12_Control
        TX_Reg[2] = data & 0xff;
        I2C_WriteReg(0x3E, TX_Reg, 3);
        delayUS(1000);
        TX_Buffer[0] = Checksum(TX_Reg, 3);
        TX_Buffer[1] = 0x05;  //combined length of registers address and data
        I2C_WriteReg(0x60, TX_Buffer, 2);
        delayUS(1000);
    } else if (type == BQ769X2_PROTOCOL::DIR_CMD_TYPE::W2) {  //write data with 2 bytes
        //CB_Active_Cells, CB_SET_LVL
        TX_Reg[2] = data & 0xff;
        TX_Reg[3] = (data >> 8) & 0xff;
        I2C_WriteReg(0x3E, TX_Reg, 4);
        delayUS(1000);
        TX_Buffer[0] = Checksum(TX_Reg, 4);
        TX_Buffer[1] = 0x06;  //combined length of registers address and data
        I2C_WriteReg(0x60, TX_Buffer, 2);
        delayUS(1000);
    }
}

void BQ769X2_PROTOCOL::setRegister(uint16_t reg_addr, uint32_t reg_data, uint8_t datalen)
// This function will write hex 3E for the initial write for subcommands in direct memory
// and write to register hex 60 for the checksum to enter the data transmitted was correct.
// and there are different cases for the three varying data lengths.
{
    uint8_t TX_Buffer[2]  = {0x00, 0x00};
    uint8_t TX_RegData[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    //TX_RegData in little endian format
    TX_RegData[0] = reg_addr & 0xff;
    TX_RegData[1] = (reg_addr >> 8) & 0xff;
    TX_RegData[2] = reg_data & 0xff;  //1st byte of data

    switch (datalen) {
        case 1:  //1 byte datalength
            I2C_WriteReg(0x3E, TX_RegData, 3);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] =
                0x05;  //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2);  // Write the checksum and length
            delayUS(2000);
            break;
        case 2:  //2 byte datalength
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            I2C_WriteReg(0x3E, TX_RegData, 4);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 4);
            TX_Buffer[1] =
                0x06;  //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2);  // Write the checksum and length
            delayUS(2000);
            break;
        case 4:  //4 byte datalength, Only used for CCGain and Capacity Gain
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            TX_RegData[4] = (reg_data >> 16) & 0xff;
            TX_RegData[5] = (reg_data >> 24) & 0xff;
            I2C_WriteReg(0x3E, TX_RegData, 6);
            delayUS(2000);
            TX_Buffer[0] = Checksum(TX_RegData, 6);
            TX_Buffer[1] =
                0x08;  //combined length of register address and data
            I2C_WriteReg(0x60, TX_Buffer, 2);  // Write the checksum and length
            delayUS(2000);
            break;
    }
}
//************************************BQ769X2_PROTOCOL Functions*********************************
void BQ769X2_PROTOCOL::init(tGaugeApplication *pGaugeApp)
{
    uint16_t u16TempValue;
    uint8_t u8Count;

    tBattParamsConfig *pBattParamsCfg;

    pBattParamsCfg = pGaugeApp->pBattGlobalParamList[0]->pBattParamsCfg;
    // Configures all parameters in device RAM

    // Enter CONFIGUPDATE mode (Subcommand 0x0090) - It is required to be in CONFIG_UPDATE mode to program the device RAM settings
    // See TRM for full description of CONFIG_UPDATE mode
    sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::BQ769x2_RESET);
    delayUS(60000);
    sendCommandSubcommand(SET_CFGUPDATE);
    delayUS(8000);

    // After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
    // programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769X2_PROTOCOL TRM.
    // An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
    // a full description of the register and the bits will pop up on the screen.

    // 'Power Config' - 0x9234 = 0x2D80
    // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
    // Set wake speed bits to 00 for best performance
    setRegister(PowerConfig, 0x2D80, 2);

    // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
    setRegister(REG0Config, 0x01, 1);

    // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
    setRegister(REG12Config, 0x0D, 1);

    // Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
    setRegister(DFETOFFPinConfig, 0x42, 1);

    // Set up ALERT Pin - 0x92FC = 0x2A
    // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
    // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
    setRegister(ALERTPinConfig, 0x2A, 1);

    // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
    setRegister(TS1Config, 0x07, 1);

    // Set TS3 to measure FET Temperature - 0x92FF = 0x0F
    setRegister(TS3Config, 0x0F, 1);

    // Set HDQ to measure Cell Temperature - 0x9300 = 0x07
    setRegister(HDQPinConfig, 0x00,
        1);  // No thermistor installed on EVM HDQ pin, so set to 0x00

    // 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
    // Only for openwire detection and  protection
    u16TempValue = 0;
    for (u8Count = 0; u8Count < (pGaugeApp->ui8NrOfCell - 1); u8Count++) {
        u16TempValue += (0x1 << u8Count);
    }
    u16TempValue += 0x8000;
    setRegister(VCellMode, u16TempValue, 2);

    // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
    // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
    // COV (over-voltage), CUV (under-voltage)
    setRegister(EnabledProtectionsA, 0xBC, 1);

    // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
    // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
    setRegister(EnabledProtectionsB, 0xF7, 1);

    // 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
    setRegister(DefaultAlarmMask, 0xF882, 2);

    // Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
    // Also see "Cell Balancing with BQ769X2_PROTOCOL Battery Monitors" document on ti.com
    setRegister(BalancingConfiguration, 0x03, 1);

    //Set the minimum cell balance voltage in charge - 0x933B = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    setRegister(CellBalanceMinCellVCharge,
        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);
    //Set the minimum cell balance voltage in rest - 0x933F = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    setRegister(CellBalanceMinCellVRelax,
        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);

    // Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
    // CUV Threshold is this value multiplied by 50.6mV
    //    BQ769X2_PROTOCOL_SetRegister(CUVThreshold, 0x31, 1);
    setRegister(
        CUVThreshold, pBattParamsCfg->u16MinBattVoltThd_mV / 51, 1);

    // Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
    // COV Threshold is this value multiplied by 50.6mV
    //    BQ769X2_PROTOCOL_SetRegister(COVThreshold, 0x55, 1);
    setRegister(
        COVThreshold, pBattParamsCfg->u16MaxBattVoltThd_mV / 51, 1);

    // Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
    //    BQ769X2_PROTOCOL_SetRegister(OCCThreshold, 0x05, 1);
    setRegister(
        OCCThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up OCD1 (over-current in discharge) Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
    //    BQ769X2_PROTOCOL_SetRegister(OCD1Threshold, 0x0A, 1);
    setRegister(
        OCD1Threshold, pBattParamsCfg->i16MinDhgCurtThd_mA / 2000, 1);

    // Set up SCD (short discharge current) Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  0x05=100mV
    //    BQ769X2_PROTOCOL_SetRegister(SCDThreshold, 0x05, 1);
    setRegister(
        SCDThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 �s; min value of 1
    setRegister(SCDDelay, 0x03, 1);

    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    setRegister(SCDLLatchLimit, 0x01, 1);

    delayUS(8000);
    // Exit CONFIGUPDATE mode  - Subcommand 0x0092
    sendCommandSubcommand(EXIT_CFGUPDATE);
    delayUS(8000);
    //Control All FETs on
    sendCommandSubcommand(FET_ENABLE);
    delayUS(8000);
    sendCommandSubcommand(ALL_FETS_ON);
    delayUS(8000);
    sendCommandSubcommand(SLEEP_DISABLE);
    delayUS(8000);
}

// ********************************* BQ769X2_PROTOCOL Status and Fault Commands   *****************************************

void BQ769X2_PROTOCOL::readAlarmStatus()
{
    // Read this register to find out why the ALERT pin was asserted
    sendDirectCommand(AlarmStatus, 0x00, R);
    AlarmBits = (uint16_t) RX_data[1] * 256 + (uint16_t) RX_data[0];
}

void BQ769X2_PROTOCOL::readSafetyStatus()
{
    // Read Safety Status A/B/C and find which bits are set
    // This shows which primary protections have been triggered
    sendDirectCommand(SafetyStatusA, 0x00, R);
    value_SafetyStatusA = (RX_data[1] * 256 + RX_data[0]);
    //Example Fault Flags
    UV_Fault  = ((0x4 & RX_data[0]) >> 2);
    OV_Fault  = ((0x8 & RX_data[0]) >> 3);
    SCD_Fault = ((0x8 & RX_data[1]) >> 3);
    OCD_Fault = ((0x2 & RX_data[1]) >> 1);
    sendDirectCommand(SafetyStatusB, 0x00, R);
    value_SafetyStatusB = (RX_data[1] * 256 + RX_data[0]);
    sendDirectCommand(SafetyStatusC, 0x00, R);
    value_SafetyStatusC = (RX_data[1] * 256 + RX_data[0]);
    if ((value_SafetyStatusA + value_SafetyStatusB + value_SafetyStatusC) >
        1) {
        ProtectionsTriggered = 1;
    } else {
        ProtectionsTriggered = 0;
    }
}

void BQ769X2_PROTOCOL::readPFStatus()
{
    // Read Permanent Fail Status A/B/C and find which bits are set
    // This shows which permanent failures have been triggered
    sendDirectCommand(PFStatusA, 0x00, R);
    value_PFStatusA = (RX_data[1] * 256 + RX_data[0]);
    sendDirectCommand(PFStatusB, 0x00, R);
    value_PFStatusB = (RX_data[1] * 256 + RX_data[0]);
    sendDirectCommand(PFStatusC, 0x00, R);
    value_PFStatusC = (RX_data[1] * 256 + RX_data[0]);
}

void BQ769X2_PROTOCOL::readFETStatus()
{
    // Read FET Status to see which FETs are enabled
    sendDirectCommand(FETStatus, 0x00, R);
    FET_Status = (RX_data[1] * 256 + RX_data[0]);
}
// ********************************* End of BQ769X2_PROTOCOL Status and Fault Commands   *****************************************

// ********************************* BQ769X2_PROTOCOL Measurement Commands   *****************************************

uint16_t BQ769X2_PROTOCOL::readVoltage(CmdDrt command)
// This function can be used to read a specific cell voltage or stack / pack / LD voltage
{
    //RX_data is global var
    sendDirectCommand(command, 0x00, BQ769X2_PROTOCOL::DIR_CMD_TYPE::R);
    if (command >= CmdDrt::Cell1Voltage &&
        command <= CmdDrt::Cell16Voltage) {  //Cells 1 through 16 (0x14 to 0x32)
        return (RX_data[1] * 256 + RX_data[0]);  //voltage is reported in mV
    } else {                                     //stack, Pack, LD
        return 10 * (RX_data[1] * 256 +
                        RX_data[0]);  //voltage is reported in 0.01V units
    }
}

/*
void readAllVoltages()
// Reads all cell voltages, Stack voltage, PACK pin voltage, and LD pin voltage
{
    uint8_t x;
    CmdDrt cellvoltageholder = CmdDrt::Cell1Voltage;  //Cell1Voltage is 0x14
    for (x = 0; x < 16; x++) {             //Reads all cell voltages
        CellVoltage[x]    = readVoltage(cellvoltageholder);
        cellvoltageholder = (CmdDrt)(((uint8_t)cellvoltageholder) + 2);
    }
    Stack_Voltage = readVoltage(CmdDrt::StackVoltage);
    Pack_Voltage  = readVoltage(CmdDrt::PACKPinVoltage);
    LD_Voltage    = readVoltage(CmdDrt::LDPinVoltage);
}
*/

void BQ769X2_PROTOCOL::readCurrent()
// Reads PACK current
{
    sendDirectCommand(CmdDrt::CC2Current, 0x00, DIR_CMD_TYPE::R);
    Pack_Current =
        (int16_t)((uint16_t) RX_data[1] * 256 +
                  (uint16_t) RX_data[0]);  // current is reported in mA
}

float BQ769X2_PROTOCOL::readTemperature(CmdDrt command)
{
    sendDirectCommand(command, 0x00, DIR_CMD_TYPE::R);
    //RX_data is a global var
    return (0.1 * (float) (RX_data[1] * 256 + RX_data[0])) -
           273.15;  // converts from 0.1K to Celcius
}

void BQ769X2_PROTOCOL::readAllTemperatures()
{
    Temperature[0] = readTemperature(TS1Temperature);
    Temperature[1] = readTemperature(TS2Temperature);
    Temperature[2] = readTemperature(TS3Temperature);
}

void BQ769X2_PROTOCOL::readPassQ()
{  // Read Accumulated Charge and Time from DASTATUS6
    sendSubcommand(Cmd::DASTATUS6, 0x00, DIR_CMD_TYPE::R);
    AccumulatedCharge_Int  = ((RX_32Byte[3] << 24) + (RX_32Byte[2] << 16) +
                             (RX_32Byte[1] << 8) + RX_32Byte[0]);  //Bytes 0-3
    AccumulatedCharge_Frac = ((RX_32Byte[7] << 24) + (RX_32Byte[6] << 16) +
                              (RX_32Byte[5] << 8) + RX_32Byte[4]);  //Bytes 4-7
    AccumulatedCharge_Time =
        ((RX_32Byte[11] << 24) + (RX_32Byte[10] << 16) + (RX_32Byte[9] << 8) +
            RX_32Byte[8]);  //Bytes 8-11
}

//bool BQ769X2_PROTOCOL::I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout)
//{
//    timeout = pdMS_TO_TICKS(1000);
//    TickType_t timeoutTime = xTaskGetTickCount() + timeout;
//
//    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &reg_addr, count);
//
//    /* Wait for I2C to be Idle */
//    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
//        if(xTaskGetTickCount() > timeoutTime){ // check timeout
//            DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//            return false;
//        }
//    }
//
//    DL_I2C_startControllerTransfer(
//        I2C_0_INST, I2C_TARGET_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_TX, 1);
//
//    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS){
//        if(xTaskGetTickCount() > timeoutTime){ // check timeout
//            DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//            return false;
//        }
//    }
//
//    /* Wait for I2C to be Idle */
//    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
//        if(xTaskGetTickCount() > timeoutTime){ // check timeout
//            DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//            return false;
//        }
//    }
//
//    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//
//    /* Send a read request to Target */
//    DL_I2C_startControllerTransfer(
//        I2C_0_INST, I2C_TARGET_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_RX, count);
//
//    for (uint8_t i = 0; i < count; i++) {
//        while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST)) {
//            if(xTaskGetTickCount() > timeoutTime){ // check timeout
//                DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//                return false;
//            }
//        }
//
//        reg_data[i] = DL_I2C_receiveControllerData(I2C_0_INST);
//    }
//
//    return true;
//}

bool BQ769X2_PROTOCOL::I2C_ReadReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout)
{
//    timeout = pdMS_TO_TICKS(1000);
    TickType_t timeoutTime = xTaskGetTickCount() + timeout;

    // send address to fetch
    if(System::i2c1.tx_blocking(
                I2C_TARGET_ADDRESS,
                &reg_addr,
                1,
                timeout
            )){
        // success
    } else {
        return false;
    }

    // read fetched data
    if(System::i2c1.rx_blocking(
            I2C_TARGET_ADDRESS,
            reg_data,
            count,
            timeout - xTaskGetTickCount()
        )){
        return true;
    } else {
        return false;
    }

}

//bool BQ769X2_PROTOCOL::I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout)
//{
//    TickType_t timeoutTime = xTaskGetTickCount() + timeout;
//
//    uint8_t I2Ctxbuff[8] = {0x00};
//
//    I2Ctxbuff[0] = reg_addr;
//    uint8_t i, j = 1;
//
//    for (i = 0; i < count; i++) {
//        I2Ctxbuff[j] = reg_data[i];
//        j++;
//    }
//
//    //    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//    DL_I2C_fillControllerTXFIFO(I2C_0_INST, &I2Ctxbuff[0], count + 1);
//
//    /* Wait for I2C to be Idle */
//    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
//             DL_I2C_CONTROLLER_STATUS_IDLE)) {
//
//        if(xTaskGetTickCount() > timeoutTime){ // check timeout
//            DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//            return false;
//        }
//    }
//
//
//    DL_I2C_startControllerTransfer(I2C_0_INST, I2C_TARGET_ADDRESS,
//        DL_I2C_CONTROLLER_DIRECTION_TX, count + 1);
//
//    while (DL_I2C_getControllerStatus(I2C_0_INST) &
//           DL_I2C_CONTROLLER_STATUS_BUSY_BUS) {
//        if(xTaskGetTickCount() > timeoutTime){ // check timeout
//            DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//            return false;
//        }
//    }
//
//    /* Wait for I2C to be Idle */
//    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
//             DL_I2C_CONTROLLER_STATUS_IDLE)) {
//        if(xTaskGetTickCount() > timeoutTime){ // check timeout
//            DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//            return false;
//        }
//    }
//
//    //Avoid BQ769x2 to stretch the SCLK too long and generate a timeout interrupt at 400kHz because of low power mode
//    // if(DL_I2C_getRawInterruptStatus(I2C_0_INST,DL_I2C_INTERRUPT_CONTROLLER_CLOCK_TIMEOUT))
//    // {
//    //     DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//    //     DL_I2C_clearInterruptStatus(I2C_0_INST,DL_I2C_INTERRUPT_CONTROLLER_CLOCK_TIMEOUT);
//    //     I2C_WriteReg(reg_addr, reg_data, count);
//    // }
//    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//    return true;
//}

bool BQ769X2_PROTOCOL::I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout)
{
    TickType_t timeoutTime = xTaskGetTickCount() + timeout;

    uint8_t I2Ctxbuff[8] = {0x00};

    I2Ctxbuff[0] = reg_addr;
    uint8_t i, j = 1;

    for (i = 0; i < count; i++) {
        I2Ctxbuff[j] = reg_data[i];
        j++;
    }

    if(System::i2c1.tx_blocking(
            I2C_TARGET_ADDRESS,
            I2Ctxbuff,
            count + 1,
            timeout
        )){
        return true;
    } else {
        return false;
    }

}

//************************************End of BQ769X2_PROTOCOL Measurement Commands******************************************
