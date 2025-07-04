/* george note :
 *  this all fuged up and what not.
 *
 *  - baselined from TI's c library, made it oop-ish so we can support
 *      multiple chips, also uses our DL wrappers
 */

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

#ifndef BQ769X2_PROTOCOL_H_
#define BQ769X2_PROTOCOL_H_

#include "Core/system.hpp"
#include "Middleware/I2C_communication.hpp"
#include "ti/battery_gauge/gauge_level2/Gauge_Type.h"

//BQ769x2 General Program Header File
#define CRC_Mode 0  // 0 for disabled, 1 for enabled
#define R 0         //Read
#define W 1         //Write
#define W2 2        //write data with two bytes

class BQ769x2 {
public:
    //Data  Memory  registers   Name in TRM
    static constexpr uint16_t Cell1Gain =               0x9180;     //Calibration:Voltage:Cell 1 Gain
    static constexpr uint16_t Cell2Gain =               0x9182;     //Calibration:Voltage:Cell 2 Gain
    static constexpr uint16_t Cell3Gain =               0x9184;     //Calibration:Voltage:Cell 3 Gain
    static constexpr uint16_t Cell4Gain =               0x9186;     //Calibration:Voltage:Cell 4 Gain
    static constexpr uint16_t Cell5Gain =               0x9188;     //Calibration:Voltage:Cell 5 Gain
    static constexpr uint16_t Cell6Gain =               0x918A;     //Calibration:Voltage:Cell 6 Gain
    static constexpr uint16_t Cell7Gain =               0x918C;     //Calibration:Voltage:Cell 7 Gain
    static constexpr uint16_t Cell8Gain =               0x918E;     //Calibration:Voltage:Cell 8 Gain
    static constexpr uint16_t Cell9Gain =               0x9190;     //Calibration:Voltage:Cell 9 Gain
    static constexpr uint16_t Cell10Gain =                  0x9192;    //Calibration:Voltage:Cell 10 Gain
    static constexpr uint16_t Cell11Gain =                  0x9194;    //Calibration:Voltage:Cell 11 Gain
    static constexpr uint16_t Cell12Gain =                  0x9196;    //Calibration:Voltage:Cell 12 Gain
    static constexpr uint16_t Cell13Gain =                  0x9198;    //Calibration:Voltage:Cell 13 Gain
    static constexpr uint16_t Cell14Gain =                  0x919A;    //Calibration:Voltage:Cell 14 Gain
    static constexpr uint16_t Cell15Gain =                  0x919C;    //Calibration:Voltage:Cell 15 Gain
    static constexpr uint16_t Cell16Gain =                  0x919E;    //Calibration:Voltage:Cell 16 Gain
    static constexpr uint16_t PackGain =                0x91A0;      //Calibration:Voltage:Pack Gain
    static constexpr uint16_t TOSGain =                 0x91A2;       //Calibration:Voltage:TOS Gain
    static constexpr uint16_t LDGain =                  0x91A4;        //Calibration:Voltage:LD Gain
    static constexpr uint16_t ADCGain =                 0x91A6;       //Calibration:Voltage:ADC Gain
    static constexpr uint16_t CCGain =                  0x91A8;        //Calibration:Current:CC Gain
    static constexpr uint16_t CapacityGain =                0x91AC;  //Calibration:Current:Capacity Gain
    static constexpr uint16_t VcellOffset =                 0x91B0;   //Calibration:Vcell Offset:Vcell Offset
    static constexpr uint16_t VdivOffset =                  0x91B2;    //Calibration:V Divider Offset:Vdiv Offset
    static constexpr uint16_t CoulombCounterOffsetSamples =                 0x91C6;  //Calibration:Current Offset:Coulomb Counter Offset Samples
    static constexpr uint16_t BoardOffset =                 0x91C8;  //Calibration:Current Offset:Board Offset
    static constexpr uint16_t InternalTempOffset =                  0x91CA;  //Calibration:Temperature:Internal Temp Offset
    static constexpr uint16_t CFETOFFTempOffset =               0x91CB;  //Calibration:Temperature:CFETOFF Temp Offset
    static constexpr uint16_t DFETOFFTempOffset =               0x91CC;  //Calibration:Temperature:DFETOFF Temp Offset
    static constexpr uint16_t ALERTTempOffset =                 0x91CD;    //Calibration:Temperature:ALERT Temp Offset
    static constexpr uint16_t TS1TempOffset =               0x91CE;      //Calibration:Temperature:TS1 Temp Offset
    static constexpr uint16_t TS2TempOffset =               0x91CF;      //Calibration:Temperature:TS2 Temp Offset
    static constexpr uint16_t TS3TempOffset =               0x91D0;      //Calibration:Temperature:TS3 Temp Offset
    static constexpr uint16_t HDQTempOffset =               0x91D1;      //Calibration:Temperature:HDQ Temp Offset
    static constexpr uint16_t DCHGTempOffset =                  0x91D2;     //Calibration:Temperature:DCHG Temp Offset
    static constexpr uint16_t DDSGTempOffset =                  0x91D3;     //Calibration:Temperature:DDSG Temp Offset
    static constexpr uint16_t IntGain =                 0x91E2;            //Calibration:Internal Temp Model:Int Gain
    static constexpr uint16_t Intbaseoffset =               0x91E4;  //Calibration:Internal Temp Model:Int base offset
    static constexpr uint16_t IntMaximumAD =                0x91E6;   //Calibration:Internal Temp Model:Int Maximum AD
    static constexpr uint16_t IntMaximumTemp =                  0x91E8;                   //Calibration:Internal Temp Model:Int Maximum Temp
    static constexpr uint16_t T18kCoeffa1 =                 0x91EA;   //Calibration:18K Temperature Model:Coeff a1
    static constexpr uint16_t T18kCoeffa2 =                 0x91EC;   //Calibration:18K Temperature Model:Coeff a2
    static constexpr uint16_t T18kCoeffa3 =                 0x91EE;   //Calibration:18K Temperature Model:Coeff a3
    static constexpr uint16_t T18kCoeffa4 =                 0x91F0;   //Calibration:18K Temperature Model:Coeff a4
    static constexpr uint16_t T18kCoeffa5 =                 0x91F2;   //Calibration:18K Temperature Model:Coeff a5
    static constexpr uint16_t T18kCoeffb1 =                 0x91F4;   //Calibration:18K Temperature Model:Coeff b1
    static constexpr uint16_t T18kCoeffb2 =                 0x91F6;   //Calibration:18K Temperature Model:Coeff b2
    static constexpr uint16_t T18kCoeffb3 =                 0x91F8;   //Calibration:18K Temperature Model:Coeff b3
    static constexpr uint16_t T18kCoeffb4 =                 0x91FA;   //Calibration:18K Temperature Model:Coeff b4
    static constexpr uint16_t T18kAdc0 =                0x91FE;      //Calibration:18K Temperature Model:Adc0
    static constexpr uint16_t T180kCoeffa1 =                0x9200;  //Calibration:180K Temperature Model:Coeff a1
    static constexpr uint16_t T180kCoeffa2 =                0x9202;  //Calibration:180K Temperature Model:Coeff a2
    static constexpr uint16_t T180kCoeffa3 =                0x9204;  //Calibration:180K Temperature Model:Coeff a3
    static constexpr uint16_t T180kCoeffa4 =                0x9206;  //Calibration:180K Temperature Model:Coeff a4
    static constexpr uint16_t T180kCoeffa5 =                0x9208;  //Calibration:180K Temperature Model:Coeff a5
    static constexpr uint16_t T180kCoeffb1 =                0x920A;  //Calibration:180K Temperature Model:Coeff b1
    static constexpr uint16_t T180kCoeffb2 =                0x920C;  //Calibration:180K Temperature Model:Coeff b2
    static constexpr uint16_t T180kCoeffb3 =                0x920E;  //Calibration:180K Temperature Model:Coeff b3
    static constexpr uint16_t T180kCoeffb4 =                0x9210;  //Calibration:180K Temperature Model:Coeff b4
    static constexpr uint16_t T180kAdc0 =               0x9214;     //Calibration:180K Temperature Model:Adc0
    static constexpr uint16_t CustomCoeffa1 =               0x9216;  //Calibration:Custom Temperature Model:Coeff a1
    static constexpr uint16_t CustomCoeffa2 =               0x9218;  //Calibration:Custom Temperature Model:Coeff a2
    static constexpr uint16_t CustomCoeffa3 =               0x921A;  //Calibration:Custom Temperature Model:Coeff a3
    static constexpr uint16_t CustomCoeffa4 =               0x921C;  //Calibration:Custom Temperature Model:Coeff a4
    static constexpr uint16_t CustomCoeffa5 =               0x921E;  //Calibration:Custom Temperature Model:Coeff a5
    static constexpr uint16_t CustomCoeffb1 =               0x9220;  //Calibration:Custom Temperature Model:Coeff b1
    static constexpr uint16_t CustomCoeffb2 =               0x9222;  //Calibration:Custom Temperature Model:Coeff b2
    static constexpr uint16_t CustomCoeffb3 =               0x9224;  //Calibration:Custom Temperature Model:Coeff b3
    static constexpr uint16_t CustomCoeffb4 =               0x9226;  //Calibration:Custom Temperature Model:Coeff b4
    static constexpr uint16_t CustomRc0 =               0x9228;      //Calibration:Custom Temperature Model:Rc0
    static constexpr uint16_t CustomAdc0 =                  0x922A;     //Calibration:Custom Temperature Model:Adc0
    static constexpr uint16_t CoulombCounterDeadband =                  0x922D;  //Calibration:Current Deadband:Coulomb Counter Deadband
    static constexpr uint16_t CUVThresholdOverride =                0x91D4;  //Calibration:CUV:CUV Threshold Override
    static constexpr uint16_t COVThresholdOverride =                0x91D6;  //Calibration:COV:COV Threshold Override
    static constexpr uint16_t MinBlowFuseVoltage =                  0x9231;    //Settings:Fuse:Min Blow Fuse Voltage
    static constexpr uint16_t FuseBlowTimeout =                 0x9233;       //Settings:Fuse:Fuse Blow Timeout
    static constexpr uint16_t PowerConfig =                 0x9234;           //Settings:Configuration:Power Config
    static constexpr uint16_t REG12Config =                 0x9236;           //Settings:Configuration:REG12 Config
    static constexpr uint16_t REG0Config =                  0x9237;            //Settings:Configuration:REG0 Config
    static constexpr uint16_t HWDRegulatorOptions =                 0x9238;                       //Settings:Configuration:HWD Regulator Options
    static constexpr uint16_t CommType =                0x9239;          //Settings:Configuration:Comm Type
    static constexpr uint16_t I2CAddress =                  0x923A;        //Settings:Configuration:I2C Address
    static constexpr uint16_t SPIConfiguration =                0x923C;  //Settings:Configuration:SPI Configuration
    static constexpr uint16_t CommIdleTime =                0x923D;      //Settings:Configuration:Comm Idle Time
    static constexpr uint16_t CFETOFFPinConfig =                0x92FA;  //Settings:Configuration:CFETOFF Pin Config
    static constexpr uint16_t DFETOFFPinConfig =                0x92FB;  //Settings:Configuration:DFETOFF Pin Config
    static constexpr uint16_t ALERTPinConfig =                  0x92FC;    //Settings:Configuration:ALERT Pin Config
    static constexpr uint16_t TS1Config =               0x92FD;         //Settings:Configuration:TS1 Config
    static constexpr uint16_t TS2Config =               0x92FE;         //Settings:Configuration:TS2 Config
    static constexpr uint16_t TS3Config =               0x92FF;         //Settings:Configuration:TS3 Config
    static constexpr uint16_t HDQPinConfig =                0x9300;      //Settings:Configuration:HDQ Pin Config
    static constexpr uint16_t DCHGPinConfig =               0x9301;     //Settings:Configuration:DCHG Pin Config
    static constexpr uint16_t DDSGPinConfig =               0x9302;     //Settings:Configuration:DDSG Pin Config
    static constexpr uint16_t DAConfiguration =                 0x9303;   //Settings:Configuration:DA Configuration
    static constexpr uint16_t VCellMode =               0x9304;         //Settings:Configuration:Vcell Mode
    static constexpr uint16_t CC3Samples =                  0x9307;        //Settings:Configuration:CC3 Samples
    static constexpr uint16_t ProtectionConfiguration =                 0x925F;  //Settings:Protection:Protection Configuration
    static constexpr uint16_t EnabledProtectionsA =                 0x9261;  //Settings:Protection:Enabled Protections A
    static constexpr uint16_t EnabledProtectionsB =                 0x9262;  //Settings:Protection:Enabled Protections B
    static constexpr uint16_t EnabledProtectionsC =                 0x9263;  //Settings:Protection:Enabled Protections C
    static constexpr uint16_t CHGFETProtectionsA =                  0x9265;   //Settings:Protection:CHG FET Protections A
    static constexpr uint16_t CHGFETProtectionsB =                  0x9266;   //Settings:Protection:CHG FET Protections B
    static constexpr uint16_t CHGFETProtectionsC =                  0x9267;   //Settings:Protection:CHG FET Protections C
    static constexpr uint16_t DSGFETProtectionsA =                  0x9269;   //Settings:Protection:DSG FET Protections A
    static constexpr uint16_t DSGFETProtectionsB =                  0x926A;   //Settings:Protection:DSG FET Protections B
    static constexpr uint16_t DSGFETProtectionsC =                  0x926B;   //Settings:Protection:DSG FET Protections C
    static constexpr uint16_t BodyDiodeThreshold =                  0x9273;   //Settings:Protection:Body Diode Threshold
    static constexpr uint16_t DefaultAlarmMask =                0x926D;     //Settings:Alarm:Default Alarm Mask
    static constexpr uint16_t SFAlertMaskA =                0x926F;         //Settings:Alarm:SF Alert Mask A
    static constexpr uint16_t SFAlertMaskB =                0x9270;         //Settings:Alarm:SF Alert Mask B
    static constexpr uint16_t SFAlertMaskC =                0x9271;         //Settings:Alarm:SF Alert Mask C
    static constexpr uint16_t PFAlertMaskA =                0x92C4;         //Settings:Alarm:PF Alert Mask A
    static constexpr uint16_t PFAlertMaskB =                0x92C5;         //Settings:Alarm:PF Alert Mask B
    static constexpr uint16_t PFAlertMaskC =                0x92C6;         //Settings:Alarm:PF Alert Mask C
    static constexpr uint16_t PFAlertMaskD =                0x92C7;         //Settings:Alarm:PF Alert Mask D
    static constexpr uint16_t EnabledPFA =                  0x92C0;           //Settings:Permanent Failure:Enabled PF A
    static constexpr uint16_t EnabledPFB =                  0x92C1;           //Settings:Permanent Failure:Enabled PF B
    static constexpr uint16_t EnabledPFC =                  0x92C2;           //Settings:Permanent Failure:Enabled PF C
    static constexpr uint16_t EnabledPFD =                  0x92C3;           //Settings:Permanent Failure:Enabled PF D
    static constexpr uint16_t FETOptions =                  0x9308;           //Settings:FET:FET Options
    static constexpr uint16_t ChgPumpControl =                  0x9309;       //Settings:FET:Chg Pump Control
    static constexpr uint16_t PrechargeStartVoltage =               0x930A;  //Settings:FET:Precharge Start Voltage
    static constexpr uint16_t PrechargeStopVoltage =                0x930C;   //Settings:FET:Precharge Stop Voltage
    static constexpr uint16_t PredischargeTimeout =                 0x930E;    //Settings:FET:Predischarge Timeout
    static constexpr uint16_t PredischargeStopDelta =               0x930F;  //Settings:FET:Predischarge Stop Delta
    static constexpr uint16_t DsgCurrentThreshold =                 0x9310;  //Settings:Current Thresholds:Dsg Current Threshold
    static constexpr uint16_t ChgCurrentThreshold =                 0x9312;                //Settings:Current Thresholds:Chg Current Threshold
    static constexpr uint16_t CheckTime =               0x9314;  //Settings:Cell Open-Wire:Check Time
    static constexpr uint16_t Cell1Interconnect =  0x9315;  //Settings:Interconnect Resistances:Cell 1 Interconnect
    static constexpr uint16_t Cell2Interconnect =  0x9317;  //Settings:Interconnect Resistances:Cell 2 Interconnect
    static constexpr uint16_t Cell3Interconnect =  0x9319;  //Settings:Interconnect Resistances:Cell 3 Interconnect
    static constexpr uint16_t Cell4Interconnect =  0x931B;  //Settings:Interconnect Resistances:Cell 4 Interconnect
    static constexpr uint16_t Cell5Interconnect =  0x931D;  //Settings:Interconnect Resistances:Cell 5 Interconnect
    static constexpr uint16_t Cell6Interconnect =  0x931F;  //Settings:Interconnect Resistances:Cell 6 Interconnect
    static constexpr uint16_t Cell7Interconnect =  0x9321;  //Settings:Interconnect Resistances:Cell 7 Interconnect
    static constexpr uint16_t Cell8Interconnect =  0x9323;  //Settings:Interconnect Resistances:Cell 8 Interconnect
    static constexpr uint16_t Cell9Interconnect =  0x9325;  //Settings:Interconnect Resistances:Cell 9 Interconnect
    static constexpr uint16_t Cell10Interconnect =  0x9327;  //Settings:Interconnect Resistances:Cell 10 Interconnect
    static constexpr uint16_t Cell11Interconnect =  0x9329;  //Settings:Interconnect Resistances:Cell 11 Interconnect
    static constexpr uint16_t Cell12Interconnect =  0x932B;  //Settings:Interconnect Resistances:Cell 12 Interconnect
    static constexpr uint16_t Cell13Interconnect =  0x932D;  //Settings:Interconnect Resistances:Cell 13 Interconnect
    static constexpr uint16_t Cell14Interconnect =  0x932F;  //Settings:Interconnect Resistances:Cell 14 Interconnect
    static constexpr uint16_t Cell15Interconnect =  0x9331;  //Settings:Interconnect Resistances:Cell 15 Interconnect
    static constexpr uint16_t Cell16Interconnect =  0x9333;  //Settings:Interconnect Resistances:Cell 16 Interconnect
    static constexpr uint16_t MfgStatusInit =  0x9343;  //Settings:Manufacturing:Mfg Status Init
    static constexpr uint16_t BalancingConfiguration =  0x9335;  //Settings:Cell Balancing Config:Balancing Configuration
    static constexpr uint16_t MinCellTemp =  0x9336;  //Settings:Cell Balancing Config:Min Cell Temp
    static constexpr uint16_t MaxCellTemp =  0x9337;  //Settings:Cell Balancing Config:Max Cell Temp
    static constexpr uint16_t MaxInternalTemp =  0x9338;  //Settings:Cell Balancing Config:Max Internal Temp
    static constexpr uint16_t CellBalanceInterval =  0x9339;  //Settings:Cell Balancing Config:Cell Balance Interval
    static constexpr uint16_t CellBalanceMaxCells =  0x933A;  //Settings:Cell Balancing Config:Cell Balance Max Cells
    static constexpr uint16_t CellBalanceMinCellVCharge =  0x933B;  //Settings:Cell Balancing Config:Cell Balance Min Cell V (Charge)
    static constexpr uint16_t CellBalanceMinDeltaCharge =  0x933D;  //Settings:Cell Balancing Config:Cell Balance Min Delta (Charge)
    static constexpr uint16_t CellBalanceStopDeltaCharge =  0x933E;  //Settings:Cell Balancing Config:Cell Balance Stop Delta (Charge)
    static constexpr uint16_t CellBalanceMinCellVRelax =  0x933F;  //Settings:Cell Balancing Config:Cell Balance Min Cell V (Relax)
    static constexpr uint16_t CellBalanceMinDeltaRelax =  0x9341;  //Settings:Cell Balancing Config:Cell Balance Min Delta (Relax)
    static constexpr uint16_t CellBalanceStopDeltaRelax =  0x9342;  //Settings:Cell Balancing Config:Cell Balance Stop Delta (Relax)
    static constexpr uint16_t ShutdownCellVoltage =  0x923F;   //Power:Shutdown:Shutdown Cell Voltage
    static constexpr uint16_t ShutdownStackVoltage =  0x9241;  //Power:Shutdown:Shutdown Stack Voltage
    static constexpr uint16_t LowVShutdownDelay =  0x9243;     //Power:Shutdown:Low V Shutdown Delay
    static constexpr uint16_t ShutdownTemperature =  0x9244;   //Power:Shutdown:Shutdown Temperature
    static constexpr uint16_t ShutdownTemperatureDelay =  0x9245;                  //Power:Shutdown:Shutdown Temperature Delay
    static constexpr uint16_t FETOffDelay =  0x9252;  //Power:Shutdown:FET Off Delay
    static constexpr uint16_t ShutdownCommandDelay =  0x9253;   //Power:Shutdown:Shutdown Command Delay
    static constexpr uint16_t AutoShutdownTime =  0x9254;       //Power:Shutdown:Auto Shutdown Time
    static constexpr uint16_t RAMFailShutdownTime =  0x9255;    //Power:Shutdown:RAM Fail Shutdown Time
    static constexpr uint16_t SleepCurrent =  0x9248;           //Power:Sleep:Sleep Current
    static constexpr uint16_t VoltageTime =  0x924A;            //Power:Sleep:Voltage Time
    static constexpr uint16_t WakeComparatorCurrent =  0x924B;  //Power:Sleep:Wake Comparator Current
    static constexpr uint16_t SleepHysteresisTime =  0x924D;    //Power:Sleep:Sleep Hysteresis Time
    static constexpr uint16_t SleepChargerVoltageThreshold =  0x924E;  //Power:Sleep:Sleep Charger Voltage Threshold
    static constexpr uint16_t SleepChargerPACKTOSDelta =  0x9250;                         //Power:Sleep:Sleep Charger PACK-TOS Delta
    static constexpr uint16_t ConfigRAMSignature =  0x91E0;  //System Data:Integrity:Config RAM Signature
    static constexpr uint16_t CUVThreshold =  0x9275;        //Protections:CUV:Threshold
    static constexpr uint16_t CUVDelay =  0x9276;            //Protections:CUV:Delay
    static constexpr uint16_t CUVRecoveryHysteresis =  0x927B;    //Protections:CUV:Recovery Hysteresis
    static constexpr uint16_t COVThreshold =  0x9278;             //Protections:COV:Threshold
    static constexpr uint16_t COVDelay =  0x9279;                 //Protections:COV:Delay
    static constexpr uint16_t COVRecoveryHysteresis =  0x927C;    //Protections:COV:Recovery Hysteresis
    static constexpr uint16_t COVLLatchLimit =  0x927D;           //Protections:COVL:Latch Limit
    static constexpr uint16_t COVLCounterDecDelay =  0x927E;      //Protections:COVL:Counter Dec Delay
    static constexpr uint16_t COVLRecoveryTime =  0x927F;         //Protections:COVL:Recovery Time
    static constexpr uint16_t OCCThreshold =  0x9280;             //Protections:OCC:Threshold
    static constexpr uint16_t OCCDelay =  0x9281;                 //Protections:OCC:Delay
    static constexpr uint16_t OCCRecoveryThreshold =  0x9288;     //Protections:OCC:Recovery Threshold
    static constexpr uint16_t OCCPACKTOSDelta =  0x92B0;          //Protections:OCC:PACK-TOS Delta
    static constexpr uint16_t OCD1Threshold =  0x9282;            //Protections:OCD1:Threshold
    static constexpr uint16_t OCD1Delay =  0x9283;                //Protections:OCD1:Delay
    static constexpr uint16_t OCD2Threshold =  0x9284;            //Protections:OCD2:Threshold
    static constexpr uint16_t OCD2Delay =  0x9285;                //Protections:OCD2:Delay
    static constexpr uint16_t SCDThreshold =  0x9286;             //Protections:SCD:Threshold
    static constexpr uint16_t SCDDelay =  0x9287;                 //Protections:SCD:Delay
    static constexpr uint16_t SCDRecoveryTime =  0x9294;          //Protections:SCD:Recovery Time
    static constexpr uint16_t OCD3Threshold =  0x928A;            //Protections:OCD3:Threshold
    static constexpr uint16_t OCD3Delay =  0x928C;                //Protections:OCD3:Delay
    static constexpr uint16_t OCDRecoveryThreshold =  0x928D;     //Protections:OCD:Recovery Threshold
    static constexpr uint16_t OCDLLatchLimit =  0x928F;           //Protections:OCDL:Latch Limit
    static constexpr uint16_t OCDLCounterDecDelay =  0x9290;      //Protections:OCDL:Counter Dec Delay
    static constexpr uint16_t OCDLRecoveryTime =  0x9291;         //Protections:OCDL:Recovery Time
    static constexpr uint16_t OCDLRecoveryThreshold =  0x9292;    //Protections:OCDL:Recovery Threshold
    static constexpr uint16_t SCDLLatchLimit =  0x9295;           //Protections:SCDL:Latch Limit
    static constexpr uint16_t SCDLCounterDecDelay =  0x9296;      //Protections:SCDL:Counter Dec Delay
    static constexpr uint16_t SCDLRecoveryTime =  0x9297;         //Protections:SCDL:Recovery Time
    static constexpr uint16_t SCDLRecoveryThreshold =  0x9298;    //Protections:SCDL:Recovery Threshold
    static constexpr uint16_t OTCThreshold =  0x929A;             //Protections:OTC:Threshold
    static constexpr uint16_t OTCDelay =  0x920B;                 //Protections:OTC:Delay
    static constexpr uint16_t OTCRecovery =  0x929C;              //Protections:OTC:Recovery
    static constexpr uint16_t OTDThreshold =  0x929D;             //Protections:OTD:Threshold
    static constexpr uint16_t OTDDelay =  0x929E;                 //Protections:OTD:Delay
    static constexpr uint16_t OTDRecovery =  0x929F;              //Protections:OTD:Recovery
    static constexpr uint16_t OTFThreshold =  0x92A0;             //Protections:OTF:Threshold
    static constexpr uint16_t OTFDelay =  0x92A1;                 //Protections:OTF:Delay
    static constexpr uint16_t OTFRecovery =  0x92A2;              //Protections:OTF:Recovery
    static constexpr uint16_t OTINTThreshold =  0x92A3;           //Protections:OTINT:Threshold
    static constexpr uint16_t OTINTDelay =  0x92A4;               //Protections:OTINT:Delay
    static constexpr uint16_t OTINTRecovery =  0x92A5;            //Protections:OTINT:Recovery
    static constexpr uint16_t UTCThreshold =  0x92A6;             //Protections:UTC:Threshold
    static constexpr uint16_t UTCDelay =  0x92A7;                 //Protections:UTC:Delay
    static constexpr uint16_t UTCRecovery =  0x92A8;              //Protections:UTC:Recovery
    static constexpr uint16_t UTDThreshold =  0x92A9;             //Protections:UTD:Threshold
    static constexpr uint16_t UTDDelay =  0x92AA;                 //Protections:UTD:Delay
    static constexpr uint16_t UTDRecovery =  0x92AB;              //Protections:UTD:Recovery
    static constexpr uint16_t UTINTThreshold =  0x92AC;           //Protections:UTINT:Threshold
    static constexpr uint16_t UTINTDelay =  0x92AD;               //Protections:UTINT:Delay
    static constexpr uint16_t UTINTRecovery =  0x92AE;            //Protections:UTINT:Recovery
    static constexpr uint16_t ProtectionsRecoveryTime =  0x92AF;  //Protections:Recovery:Time
    static constexpr uint16_t HWDDelay =  0x92B2;                 //Protections:HWD:Delay
    static constexpr uint16_t LoadDetectActiveTime =  0x92B4;     //Protections:Load Detect:Active Time
    static constexpr uint16_t LoadDetectRetryDelay =  0x92B5;     //Protections:Load Detect:Retry Delay
    static constexpr uint16_t LoadDetectTimeout =  0x92B6;        //Protections:Load Detect:Timeout
    static constexpr uint16_t PTOChargeThreshold =  0x92BA;       //Protections:PTO:Charge Threshold
    static constexpr uint16_t PTODelay =  0x92BC;                 //Protections:PTO:Delay
    static constexpr uint16_t PTOReset =  0x92BE;                 //Protections:PTO:Reset
    static constexpr uint16_t CUDEPThreshold =  0x92C8;           //Permanent Fail:CUDEP:Threshold
    static constexpr uint16_t CUDEPDelay =  0x92CA;               //Permanent Fail:CUDEP:Delay
    static constexpr uint16_t SUVThreshold =  0x92CB;             //Permanent Fail:SUV:Threshold
    static constexpr uint16_t SUVDelay =  0x92CD;                 //Permanent Fail:SUV:Delay
    static constexpr uint16_t SOVThreshold =  0x92CE;             //Permanent Fail:SOV:Threshold
    static constexpr uint16_t SOVDelay =  0x92D0;                 //Permanent Fail:SOV:Delay
    static constexpr uint16_t TOSSThreshold =  0x92D1;            //Permanent Fail:TOS:Threshold
    static constexpr uint16_t TOSSDelay =  0x92D3;                //Permanent Fail:TOS:Delay
    static constexpr uint16_t SOCCThreshold =  0x92D4;            //Permanent Fail:SOCC:Threshold
    static constexpr uint16_t SOCCDelay =  0x92D6;                //Permanent Fail:SOCC:Delay
    static constexpr uint16_t SOCDThreshold =  0x92D7;            //Permanent Fail:SOCD:Threshold
    static constexpr uint16_t SOCDDelay =  0x92D9;                //Permanent Fail:SOCD:Delay
    static constexpr uint16_t SOTThreshold =  0x92DA;             //Permanent Fail:SOT:Threshold
    static constexpr uint16_t SOTDelay =  0x92DB;                 //Permanent Fail:SOT:Delay
    static constexpr uint16_t SOTFThreshold =  0x92DC;            //Permanent Fail:SOTF:Threshold
    static constexpr uint16_t SOTFDelay =  0x92DD;                //Permanent Fail:SOTF:Delay
    static constexpr uint16_t VIMRCheckVoltage =  0x92DE;         //Permanent Fail:VIMR:Check Voltage
    static constexpr uint16_t VIMRMaxRelaxCurrent =  0x92E0;      //Permanent Fail:VIMR:Max Relax Current
    static constexpr uint16_t VIMRThreshold =  0x92E2;            //Permanent Fail:VIMR:Threshold
    static constexpr uint16_t VIMRDelay =  0x92E4;                //Permanent Fail:VIMR:Delay
    static constexpr uint16_t VIMRRelaxMinDuration =  0x92E5;  //Permanent Fail:VIMR:Relax Min Duration
    static constexpr uint16_t VIMACheckVoltage =  0x92E7;      //Permanent Fail:VIMA:Check Voltage
    static constexpr uint16_t VIMAMinActiveCurrent =  0x92E9;  //Permanent Fail:VIMA:Min Active Current
    static constexpr uint16_t VIMAThreshold =  0x92EB;         //Permanent Fail:VIMA:Threshold
    static constexpr uint16_t VIMADelay =  0x92ED;             //Permanent Fail:VIMA:Delay
    static constexpr uint16_t CFETFOFFThreshold =  0x92EE;     //Permanent Fail:CFETF:OFF Threshold
    static constexpr uint16_t CFETFOFFDelay =  0x92F0;         //Permanent Fail:CFETF:OFF Delay
    static constexpr uint16_t DFETFOFFThreshold =  0x92F1;     //Permanent Fail:DFETF:OFF Threshold
    static constexpr uint16_t DFETFOFFDelay =  0x92F3;         //Permanent Fail:DFETF:OFF Delay
    static constexpr uint16_t VSSFFailThreshold =  0x92F4;     //Permanent Fail:VSSF:Fail Threshold
    static constexpr uint16_t VSSFDelay =  0x92F6;             //Permanent Fail:VSSF:Delay
    static constexpr uint16_t PF2LVLDelay =  0x92F7;           //Permanent Fail:2LVL:Delay
    static constexpr uint16_t LFOFDelay =  0x92F8;             //Permanent Fail:LFOF:Delay
    static constexpr uint16_t HWMXDelay =  0x92F9;             //Permanent Fail:HWMX:Delay
    static constexpr uint16_t SecuritySettings =  0x9256;      //Security:Settings:Security Settings
    static constexpr uint16_t UnsealKeyStep1 =  0x9257;        //Security:Keys:Unseal Key Step 1
    static constexpr uint16_t UnsealKeyStep2 =  0x9259;        //Security:Keys:Unseal Key Step 2
    static constexpr uint16_t FullAccessKeyStep1 =  0x925B;    //Security:Keys:Full Access Key Step 1
    static constexpr uint16_t FullAccessKeyStep2 =  0x925D;    //Security:Keys:Full Access Key Step 2

    //Direct Commands
    static constexpr uint16_t ControlStatus = 0x00;
    static constexpr uint16_t SafetyAlertA = 0x02;
    static constexpr uint16_t SafetyStatusA = 0x03;
    static constexpr uint16_t SafetyAlertB = 0x04;
    static constexpr uint16_t SafetyStatusB = 0x05;
    static constexpr uint16_t SafetyAlertC = 0x06;
    static constexpr uint16_t SafetyStatusC = 0x07;
    static constexpr uint16_t PFAlertA = 0x0A;
    static constexpr uint16_t PFStatusA = 0x0B;
    static constexpr uint16_t PFAlertB = 0x0C;
    static constexpr uint16_t PFStatusB = 0x0D;
    static constexpr uint16_t PFAlertC = 0x0E;
    static constexpr uint16_t PFStatusC = 0x0F;
    static constexpr uint16_t PFAlertD = 0x10;
    static constexpr uint16_t PFStatusD = 0x11;
    static constexpr uint16_t BatteryStatus = 0x12;
    static constexpr uint16_t Cell1Voltage = 0x14;
    static constexpr uint16_t Cell2Voltage = 0x16;
    static constexpr uint16_t Cell3Voltage = 0x18;
    static constexpr uint16_t Cell4Voltage = 0x1A;
    static constexpr uint16_t Cell5Voltage = 0x1C;
    static constexpr uint16_t Cell6Voltage = 0x1E;
    static constexpr uint16_t Cell7Voltage = 0x20;
    static constexpr uint16_t Cell8Voltage = 0x22;
    static constexpr uint16_t Cell9Voltage = 0x24;
    static constexpr uint16_t Cell10Voltage = 0x26;
    static constexpr uint16_t Cell11Voltage = 0x28;
    static constexpr uint16_t Cell12Voltage = 0x2A;
    static constexpr uint16_t Cell13Voltage = 0x2C;
    static constexpr uint16_t Cell14Voltage = 0x2E;
    static constexpr uint16_t Cell15Voltage = 0x30;
    static constexpr uint16_t Cell16Voltage = 0x32;
    static constexpr uint16_t StackVoltage = 0x34;
    static constexpr uint16_t PACKPinVoltage = 0x36;
    static constexpr uint16_t LDPinVoltage = 0x38;
    static constexpr uint16_t CC2Current = 0x3A;
    static constexpr uint16_t AlarmStatus = 0x62;
    static constexpr uint16_t AlarmRawStatus = 0x64;
    static constexpr uint16_t AlarmEnable = 0x66;
    static constexpr uint16_t IntTemperature = 0x68;
    static constexpr uint16_t CFETOFFTemperature = 0x6A;
    static constexpr uint16_t DFETOFFTemperature = 0x6C;
    static constexpr uint16_t ALERTTemperature = 0x6E;
    static constexpr uint16_t TS1Temperature = 0x70;
    static constexpr uint16_t TS2Temperature = 0x72;
    static constexpr uint16_t TS3Temperature = 0x74;
    static constexpr uint16_t HDQTemperature = 0x76;
    static constexpr uint16_t DCHGTemperature = 0x78;
    static constexpr uint16_t DDSGTemperature = 0x7A;
    static constexpr uint16_t FETStatus = 0x7F;

    //Commands
    static constexpr uint16_t DEVICE_NUMBER = 0x0001;
    static constexpr uint16_t FW_VERSION = 0x0002;
    static constexpr uint16_t HW_VERSION = 0x0003;
    static constexpr uint16_t IROM_SIG = 0x0004;
    static constexpr uint16_t STATIC_CFG_SIG = 0x0005;
    static constexpr uint16_t PREV_MACWRITE = 0x0007;
    static constexpr uint16_t DROM_SIG = 0x0009;
    static constexpr uint16_t SECURITY_KEYS = 0x0035;
    static constexpr uint16_t SAVED_PF_STATUS = 0x0053;
    static constexpr uint16_t MANUFACTURINGSTATUS = 0x0057;
    static constexpr uint16_t MANU_DATA = 0x0070;
    static constexpr uint16_t DASTATUS1 = 0x0071;
    static constexpr uint16_t DASTATUS2 = 0x0072;
    static constexpr uint16_t DASTATUS3 = 0x0073;
    static constexpr uint16_t DASTATUS4 = 0x0074;
    static constexpr uint16_t DASTATUS5 = 0x0075;
    static constexpr uint16_t DASTATUS6 = 0x0076;
    static constexpr uint16_t DASTATUS7 = 0x0077;
    static constexpr uint16_t CUV_SNAPSHOT = 0x0080;
    static constexpr uint16_t COV_SNAPSHOT = 0X0081;
    static constexpr uint16_t CB_ACTIVE_CELLS = 0x0083;
    static constexpr uint16_t CB_SET_LVL = 0x0084;
    static constexpr uint16_t CBSTATUS1 = 0x0085;
    static constexpr uint16_t CBSTATUS2 = 0x0086;
    static constexpr uint16_t CBSTATUS3 = 0x0087;
    static constexpr uint16_t FET_CONTROL = 0x0097;
    static constexpr uint16_t REG12_CONTROL = 0x0098;
    static constexpr uint16_t OTP_WR_CHECK = 0x00A0;
    static constexpr uint16_t OTP_WRITE = 0x00A1;
    static constexpr uint16_t READ_CAL1 = 0xF081;
    static constexpr uint16_t CAL_CUV = 0xF090;
    static constexpr uint16_t CAL_COV = 0xF091;

    //Command Only Subcommands
    static constexpr uint16_t EXIT_DEEPSLEEP = 0x000E;
    static constexpr uint16_t DEEPSLEEP = 0x000F;
    static constexpr uint16_t SHUTDOWN = 0x0010;
    static constexpr uint16_t BQ769x2_RESET = 0x0012;  //"RESET" in documentation
    static constexpr uint16_t PDSGTEST = 0x001C;
    static constexpr uint16_t FUSE_TOGGLE = 0x001D;
    static constexpr uint16_t PCHGTEST = 0x001E;
    static constexpr uint16_t CHGTEST = 0x001F;
    static constexpr uint16_t DSGTEST = 0x0020;
    static constexpr uint16_t FET_ENABLE = 0x0022;
    static constexpr uint16_t PF_ENABLE = 0x0024;
    static constexpr uint16_t PF_RESET = 0x0029;
    static constexpr uint16_t SEAL = 0x0030;
    static constexpr uint16_t RESET_PASSQ = 0x0082;
    static constexpr uint16_t PTO_RECOVER = 0x008A;
    static constexpr uint16_t SET_CFGUPDATE = 0x0090;
    static constexpr uint16_t EXIT_CFGUPDATE = 0x0092;
    static constexpr uint16_t DSG_PDSG_OFF = 0x0093;
    static constexpr uint16_t CHG_PCHG_OFF = 0x0094;
    static constexpr uint16_t ALL_FETS_OFF = 0x0095;
    static constexpr uint16_t ALL_FETS_ON = 0x0096;
    static constexpr uint16_t SLEEP_ENABLE = 0x0099;
    static constexpr uint16_t SLEEP_DISABLE = 0x009A;
    static constexpr uint16_t OCDL_RECOVER = 0x009B;
    static constexpr uint16_t SCDL_RECOVER = 0x009C;
    static constexpr uint16_t LOAD_DETECT_RESTART = 0x009D;
    static constexpr uint16_t LOAD_DETECT_ON = 0x009E;
    static constexpr uint16_t LOAD_DETECT_OFF = 0x009F;
    static constexpr uint16_t CFETOFF_LO = 0x2800;
    static constexpr uint16_t DFETOFF_LO = 0x2801;
    static constexpr uint16_t ALERT_LO = 0x2802;
    static constexpr uint16_t HDQ_LO = 0x2806;
    static constexpr uint16_t DCHG_LO = 0x2807;
    static constexpr uint16_t DDSG_LO = 0x2808;
    static constexpr uint16_t CFETOFF_HI = 0x2810;
    static constexpr uint16_t DFETOFF_HI = 0x2811;
    static constexpr uint16_t ALERT_HI = 0x2812;
    static constexpr uint16_t HDQ_HI = 0x2816;
    static constexpr uint16_t DCHG_HI = 0x2817;
    static constexpr uint16_t DDSG_HI = 0x2818;
    static constexpr uint16_t PF_FORCE_A = 0x2857;
    static constexpr uint16_t PF_FORCE_B = 0x29A3;
    static constexpr uint16_t SWAP_COMM_MODE = 0x29BC;
    static constexpr uint16_t SWAP_TO_I2C = 0x29E7;
    static constexpr uint16_t SWAP_TO_SPI = 0x7C35;
    static constexpr uint16_t SWAP_TO_HDQ = 0x7C40;
    static constexpr uint16_t MCR_REG = 0x1228;

    void BQ769x2_Init(tGaugeApplication *pGaugeApp);
    void BQ769x2_ReadAlarmStatus();
    void BQ769x2_ReadSafetyStatus();
    void BQ769x2_ReadPFStatus();
    uint16_t BQ769x2_ReadVoltage(uint8_t command);
    void BQ769x2_ReadAllVoltages();
    void BQ769x2_ReadCurrent();
    float BQ769x2_ReadTemperature(uint8_t command);
    void BQ769x2_ReadPassQ();
    void BQ769x2_ReadFETStatus();
    void BQ769x2_ReadAllTemperatures();
    void CommandSubcommands(uint16_t command);
    void delayUS(uint16_t us);
    void DirectCommands(uint8_t command, uint16_t data, uint8_t type);
    void BQ769x2_SetRegister(
        uint16_t reg_addr, uint32_t reg_data, uint8_t datalen);
};

#endif /* BQ769X2_PROTOCOL_H_ */
