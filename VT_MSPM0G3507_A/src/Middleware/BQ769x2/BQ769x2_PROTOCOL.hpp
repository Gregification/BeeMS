/*
 * baselined from TI's sample code found in the MSPM0 SDK.
 *      mspm0_sdk_2_05_00_05\examples\nortos\LP_MSPM0G3507\battery_gauge\gauge_level2_bq76952\Driver
 */

/*
 * see BQ769x2.TRM.3.1/13 for explanation of "Direct Commands and Subcommands"
 */

#ifndef BQ769X2_PROTOCOL_HPP_
#define BQ769X2_PROTOCOL_HPP_

#include "Core/system.hpp"
#include "ti/battery_gauge/gauge_level2/Gauge_Type.h"

namespace BQ769X2_PROTOCOL {
    struct I2C_addr {
        uint8_t read;
        uint8_t write;
    };

    //Data  Memory  registers   Name in TRM.13.9/13-37/197
    enum RegAddr : uint16_t {
        SECURITY_KEY_ENTRY_1          = 0x003E,
        SECURITY_KEY_ENTRY_2          = 0x003F,
        Cell1Gain                     = 0x9180,   //Calibration:Voltage:Cell 1 Gain
        Cell2Gain                     = 0x9182,   //Calibration:Voltage:Cell 2 Gain
        Cell3Gain                     = 0x9184,   //Calibration:Voltage:Cell 3 Gain
        Cell4Gain                     = 0x9186,   //Calibration:Voltage:Cell 4 Gain
        Cell5Gain                     = 0x9188,   //Calibration:Voltage:Cell 5 Gain
        Cell6Gain                     = 0x918A,   //Calibration:Voltage:Cell 6 Gain
        Cell7Gain                     = 0x918C,   //Calibration:Voltage:Cell 7 Gain
        Cell8Gain                     = 0x918E,   //Calibration:Voltage:Cell 8 Gain
        Cell9Gain                     = 0x9190,   //Calibration:Voltage:Cell 9 Gain
        Cell10Gain                    = 0x9192,   //Calibration:Voltage:Cell 10 Gain
        Cell11Gain                    = 0x9194,   //Calibration:Voltage:Cell 11 Gain
        Cell12Gain                    = 0x9196,   //Calibration:Voltage:Cell 12 Gain
        Cell13Gain                    = 0x9198,   //Calibration:Voltage:Cell 13 Gain
        Cell14Gain                    = 0x919A,   //Calibration:Voltage:Cell 14 Gain
        Cell15Gain                    = 0x919C,   //Calibration:Voltage:Cell 15 Gain
        Cell16Gain                    = 0x919E,   //Calibration:Voltage:Cell 16 Gain
        PackGain                      = 0x91A0,   //Calibration:Voltage:Pack Gain
        TOSGain                       = 0x91A2,   //Calibration:Voltage:TOS Gain
        LDGain                        = 0x91A4,   //Calibration:Voltage:LD Gain
        ADCGain                       = 0x91A6,   //Calibration:Voltage:ADC Gain
        CCGain                        = 0x91A8,   //Calibration:Current:CC Gain
        CapacityGain                  = 0x91AC,   //Calibration:Current:Capacity Gain
        VcellOffset                   = 0x91B0,   //Calibration:Vcell Offset:Vcell Offset
        VdivOffset                    = 0x91B2,   //Calibration:V Divider Offset:Vdiv Offset
        CoulombCounterOffsetSamples   = 0x91C6,   //Calibration:Current Offset:Coulomb Counter Offset Samples
        BoardOffset                   = 0x91C8,   //Calibration:Current Offset:Board Offset
        InternalTempOffset            = 0x91CA,   //Calibration:Temperature:Internal Temp Offset
        CFETOFFTempOffset             = 0x91CB,   //Calibration:Temperature:CFETOFF Temp Offset
        DFETOFFTempOffset             = 0x91CC,   //Calibration:Temperature:DFETOFF Temp Offset
        ALERTTempOffset               = 0x91CD,   //Calibration:Temperature:ALERT Temp Offset
        TS1TempOffset                 = 0x91CE,   //Calibration:Temperature:TS1 Temp Offset
        TS2TempOffset                 = 0x91CF,   //Calibration:Temperature:TS2 Temp Offset
        TS3TempOffset                 = 0x91D0,   //Calibration:Temperature:TS3 Temp Offset
        HDQTempOffset                 = 0x91D1,   //Calibration:Temperature:HDQ Temp Offset
        DCHGTempOffset                = 0x91D2,   //Calibration:Temperature:DCHG Temp Offset
        DDSGTempOffset                = 0x91D3,   //Calibration:Temperature:DDSG Temp Offset
        IntGain                       = 0x91E2,   //Calibration:Internal Temp Model:Int Gain
        Intbaseoffset                 = 0x91E4,   //Calibration:Internal Temp Model:Int base offset
        IntMaximumAD                  = 0x91E6,   //Calibration:Internal Temp Model:Int Maximum AD
        IntMaximumTemp                = 0x91E8,   //Calibration:Internal Temp Model:Int Maximum Temp
        T18kCoeffa1                   = 0x91EA,   //Calibration:18K Temperature Model:Coeff a1
        T18kCoeffa2                   = 0x91EC,   //Calibration:18K Temperature Model:Coeff a2
        T18kCoeffa3                   = 0x91EE,   //Calibration:18K Temperature Model:Coeff a3
        T18kCoeffa4                   = 0x91F0,   //Calibration:18K Temperature Model:Coeff a4
        T18kCoeffa5                   = 0x91F2,   //Calibration:18K Temperature Model:Coeff a5
        T18kCoeffb1                   = 0x91F4,   //Calibration:18K Temperature Model:Coeff b1
        T18kCoeffb2                   = 0x91F6,   //Calibration:18K Temperature Model:Coeff b2
        T18kCoeffb3                   = 0x91F8,   //Calibration:18K Temperature Model:Coeff b3
        T18kCoeffb4                   = 0x91FA,   //Calibration:18K Temperature Model:Coeff b4
        T18kAdc0                      = 0x91FE,   //Calibration:18K Temperature Model:Adc0
        T180kCoeffa1                  = 0x9200,   //Calibration:180K Temperature Model:Coeff a1
        T180kCoeffa2                  = 0x9202,   //Calibration:180K Temperature Model:Coeff a2
        T180kCoeffa3                  = 0x9204,   //Calibration:180K Temperature Model:Coeff a3
        T180kCoeffa4                  = 0x9206,   //Calibration:180K Temperature Model:Coeff a4
        T180kCoeffa5                  = 0x9208,   //Calibration:180K Temperature Model:Coeff a5
        T180kCoeffb1                  = 0x920A,   //Calibration:180K Temperature Model:Coeff b1
        T180kCoeffb2                  = 0x920C,   //Calibration:180K Temperature Model:Coeff b2
        T180kCoeffb3                  = 0x920E,   //Calibration:180K Temperature Model:Coeff b3
        T180kCoeffb4                  = 0x9210,   //Calibration:180K Temperature Model:Coeff b4
        T180kAdc0                     = 0x9214,   //Calibration:180K Temperature Model:Adc0
        CustomCoeffa1                 = 0x9216,   //Calibration:Custom Temperature Model:Coeff a1
        CustomCoeffa2                 = 0x9218,   //Calibration:Custom Temperature Model:Coeff a2
        CustomCoeffa3                 = 0x921A,   //Calibration:Custom Temperature Model:Coeff a3
        CustomCoeffa4                 = 0x921C,   //Calibration:Custom Temperature Model:Coeff a4
        CustomCoeffa5                 = 0x921E,   //Calibration:Custom Temperature Model:Coeff a5
        CustomCoeffb1                 = 0x9220,   //Calibration:Custom Temperature Model:Coeff b1
        CustomCoeffb2                 = 0x9222,   //Calibration:Custom Temperature Model:Coeff b2
        CustomCoeffb3                 = 0x9224,   //Calibration:Custom Temperature Model:Coeff b3
        CustomCoeffb4                 = 0x9226,   //Calibration:Custom Temperature Model:Coeff b4
        CustomRc0                     = 0x9228,   //Calibration:Custom Temperature Model:Rc0
        CustomAdc0                    = 0x922A,   //Calibration:Custom Temperature Model:Adc0
        CoulombCounterDeadband        = 0x922D,   //Calibration:Current Deadband:Coulomb Counter Deadband
        CUVThresholdOverride          = 0x91D4,   //Calibration:CUV:CUV Threshold Override
        COVThresholdOverride          = 0x91D6,   //Calibration:COV:COV Threshold Override
        MinBlowFuseVoltage            = 0x9231,   //Settings:Fuse:Min Blow Fuse Voltage
        FuseBlowTimeout               = 0x9233,   //Settings:Fuse:Fuse Blow Timeout
        PowerConfig                   = 0x9234,   //Settings:Configuration:Power Config
        REG12Config                   = 0x9236,   //Settings:Configuration:REG12 Config
        REG0Config                    = 0x9237,   //Settings:Configuration:REG0 Config
        HWDRegulatorOptions           = 0x9238,   //Settings:Configuration:HWD Regulator Options
        CommType                      = 0x9239,   //Settings:Configuration:Comm Type
        I2CAddress                    = 0x923A,   //Settings:Configuration:I2C Address
        SPIConfiguration              = 0x923C,   //Settings:Configuration:SPI Configuration
        CommIdleTime                  = 0x923D,   //Settings:Configuration:Comm Idle Time
        CFETOFFPinConfig              = 0x92FA,   //Settings:Configuration:CFETOFF Pin Config
        DFETOFFPinConfig              = 0x92FB,   //Settings:Configuration:DFETOFF Pin Config
        ALERTPinConfig                = 0x92FC,   //Settings:Configuration:ALERT Pin Config
        TS1Config                     = 0x92FD,   //Settings:Configuration:TS1 Config
        TS2Config                     = 0x92FE,   //Settings:Configuration:TS2 Config
        TS3Config                     = 0x92FF,   //Settings:Configuration:TS3 Config
        HDQPinConfig                  = 0x9300,   //Settings:Configuration:HDQ Pin Config
        DCHGPinConfig                 = 0x9301,   //Settings:Configuration:DCHG Pin Config
        DDSGPinConfig                 = 0x9302,   //Settings:Configuration:DDSG Pin Config
        DAConfiguration               = 0x9303,   //Settings:Configuration:DA Configuration
        VCellMode                     = 0x9304,   //Settings:Configuration:Vcell Mode
        CC3Samples                    = 0x9307,   //Settings:Configuration:CC3 Samples
        ProtectionConfiguration       = 0x925F,   //Settings:Protection:Protection Configuration
        EnabledProtectionsA           = 0x9261,   //Settings:Protection:Enabled Protections A
        EnabledProtectionsB           = 0x9262,   //Settings:Protection:Enabled Protections B
        EnabledProtectionsC           = 0x9263,   //Settings:Protection:Enabled Protections C
        CHGFETProtectionsA            = 0x9265,   //Settings:Protection:CHG FET Protections A
        CHGFETProtectionsB            = 0x9266,   //Settings:Protection:CHG FET Protections B
        CHGFETProtectionsC            = 0x9267,   //Settings:Protection:CHG FET Protections C
        DSGFETProtectionsA            = 0x9269,   //Settings:Protection:DSG FET Protections A
        DSGFETProtectionsB            = 0x926A,   //Settings:Protection:DSG FET Protections B
        DSGFETProtectionsC            = 0x926B,   //Settings:Protection:DSG FET Protections C
        BodyDiodeThreshold            = 0x9273,   //Settings:Protection:Body Diode Threshold
        DefaultAlarmMask              = 0x926D,   //Settings:Alarm:Default Alarm Mask
        SFAlertMaskA                  = 0x926F,   //Settings:Alarm:SF Alert Mask A
        SFAlertMaskB                  = 0x9270,   //Settings:Alarm:SF Alert Mask B
        SFAlertMaskC                  = 0x9271,   //Settings:Alarm:SF Alert Mask C
        PFAlertMaskA                  = 0x92C4,   //Settings:Alarm:PF Alert Mask A
        PFAlertMaskB                  = 0x92C5,   //Settings:Alarm:PF Alert Mask B
        PFAlertMaskC                  = 0x92C6,   //Settings:Alarm:PF Alert Mask C
        PFAlertMaskD                  = 0x92C7,   //Settings:Alarm:PF Alert Mask D
        EnabledPFA                    = 0x92C0,   //Settings:Permanent Failure:Enabled PF A
        EnabledPFB                    = 0x92C1,   //Settings:Permanent Failure:Enabled PF B
        EnabledPFC                    = 0x92C2,   //Settings:Permanent Failure:Enabled PF C
        EnabledPFD                    = 0x92C3,   //Settings:Permanent Failure:Enabled PF D
        FETOptions                    = 0x9308,   //Settings:FET:FET Options
        ChgPumpControl                = 0x9309,   //Settings:FET:Chg Pump Control
        PrechargeStartVoltage         = 0x930A,   //Settings:FET:Precharge Start Voltage
        PrechargeStopVoltage          = 0x930C,   //Settings:FET:Precharge Stop Voltage
        PredischargeTimeout           = 0x930E,   //Settings:FET:Predischarge Timeout
        PredischargeStopDelta         = 0x930F,   //Settings:FET:Predischarge Stop Delta
        DsgCurrentThreshold           = 0x9310,   //Settings:Current Thresholds:Dsg Current Threshold
        ChgCurrentThreshold           = 0x9312,   //Settings:Current Thresholds:Chg Current Threshold
        CheckTime                     = 0x9314,   //Settings:Cell Open-Wire:Check Time
        Cell1Interconnect             = 0x9315,   //Settings:Interconnect Resistances:Cell 1 Interconnect
        Cell2Interconnect             = 0x9317,   //Settings:Interconnect Resistances:Cell 2 Interconnect
        Cell3Interconnect             = 0x9319,   //Settings:Interconnect Resistances:Cell 3 Interconnect
        Cell4Interconnect             = 0x931B,   //Settings:Interconnect Resistances:Cell 4 Interconnect
        Cell5Interconnect             = 0x931D,   //Settings:Interconnect Resistances:Cell 5 Interconnect
        Cell6Interconnect             = 0x931F,   //Settings:Interconnect Resistances:Cell 6 Interconnect
        Cell7Interconnect             = 0x9321,   //Settings:Interconnect Resistances:Cell 7 Interconnect
        Cell8Interconnect             = 0x9323,   //Settings:Interconnect Resistances:Cell 8 Interconnect
        Cell9Interconnect             = 0x9325,   //Settings:Interconnect Resistances:Cell 9 Interconnect
        Cell10Interconnect            = 0x9327,   //Settings:Interconnect Resistances:Cell 10 Interconnect
        Cell11Interconnect            = 0x9329,   //Settings:Interconnect Resistances:Cell 11 Interconnect
        Cell12Interconnect            = 0x932B,   //Settings:Interconnect Resistances:Cell 12 Interconnect
        Cell13Interconnect            = 0x932D,   //Settings:Interconnect Resistances:Cell 13 Interconnect
        Cell14Interconnect            = 0x932F,   //Settings:Interconnect Resistances:Cell 14 Interconnect
        Cell15Interconnect            = 0x9331,   //Settings:Interconnect Resistances:Cell 15 Interconnect
        Cell16Interconnect            = 0x9333,   //Settings:Interconnect Resistances:Cell 16 Interconnect
        MfgStatusInit                 = 0x9343,   //Settings:Manufacturing:Mfg Status Init
        BalancingConfiguration        = 0x9335,   //Settings:Cell Balancing Config:Balancing Configuration
        MinCellTemp                   = 0x9336,   //Settings:Cell Balancing Config:Min Cell Temp
        MaxCellTemp                   = 0x9337,   //Settings:Cell Balancing Config:Max Cell Temp
        MaxInternalTemp               = 0x9338,   //Settings:Cell Balancing Config:Max Internal Temp
        CellBalanceInterval           = 0x9339,   //Settings:Cell Balancing Config:Cell Balance Interval
        CellBalanceMaxCells           = 0x933A,   //Settings:Cell Balancing Config:Cell Balance Max Cells
        CellBalanceMinCellVCharge     = 0x933B,   //Settings:Cell Balancing Config:Cell Balance Min Cell V (Charge)
        CellBalanceMinDeltaCharge     = 0x933D,   //Settings:Cell Balancing Config:Cell Balance Min Delta (Charge)
        CellBalanceStopDeltaCharge    = 0x933E,   //Settings:Cell Balancing Config:Cell Balance Stop Delta (Charge)
        CellBalanceMinCellVRelax      = 0x933F,   //Settings:Cell Balancing Config:Cell Balance Min Cell V (Relax)
        CellBalanceMinDeltaRelax      = 0x9341,   //Settings:Cell Balancing Config:Cell Balance Min Delta (Relax)
        CellBalanceStopDeltaRelax     = 0x9342,   //Settings:Cell Balancing Config:Cell Balance Stop Delta (Relax)
        ShutdownCellVoltage           = 0x923F,   //Power:Shutdown:Shutdown Cell Voltage
        ShutdownStackVoltage          = 0x9241,   //Power:Shutdown:Shutdown Stack Voltage
        LowVShutdownDelay             = 0x9243,   //Power:Shutdown:Low V Shutdown Delay
        ShutdownTemperature           = 0x9244,   //Power:Shutdown:Shutdown Temperature
        ShutdownTemperatureDelay      = 0x9245,   //Power:Shutdown:Shutdown Temperature Delay
        FETOffDelay                   = 0x9252,   //Power:Shutdown:FET Off Delay
        ShutdownCommandDelay          = 0x9253,   //Power:Shutdown:Shutdown Command Delay
        AutoShutdownTime              = 0x9254,   //Power:Shutdown:Auto Shutdown Time
        RAMFailShutdownTime           = 0x9255,   //Power:Shutdown:RAM Fail Shutdown Time
        SleepCurrent                  = 0x9248,   //Power:Sleep:Sleep Current
        VoltageTime                   = 0x924A,   //Power:Sleep:Voltage Time
        WakeComparatorCurrent         = 0x924B,   //Power:Sleep:Wake Comparator Current
        SleepHysteresisTime           = 0x924D,   //Power:Sleep:Sleep Hysteresis Time
        SleepChargerVoltageThreshold  = 0x924E,   //Power:Sleep:Sleep Charger Voltage Threshold
        SleepChargerPACKTOSDelta      = 0x9250,   //Power:Sleep:Sleep Charger PACK-TOS Delta
        ConfigRAMSignature            = 0x91E0,   //System Data:Integrity:Config RAM Signature
        CUVThreshold                  = 0x9275,   //Protections:CUV:Threshold
        CUVDelay                      = 0x9276,   //Protections:CUV:Delay
        CUVRecoveryHysteresis         = 0x927B,   //Protections:CUV:Recovery Hysteresis
        COVThreshold                  = 0x9278,   //Protections:COV:Threshold
        COVDelay                      = 0x9279,   //Protections:COV:Delay
        COVRecoveryHysteresis         = 0x927C,   //Protections:COV:Recovery Hysteresis
        COVLLatchLimit                = 0x927D,   //Protections:COVL:Latch Limit
        COVLCounterDecDelay           = 0x927E,   //Protections:COVL:Counter Dec Delay
        COVLRecoveryTime              = 0x927F,   //Protections:COVL:Recovery Time
        OCCThreshold                  = 0x9280,   //Protections:OCC:Threshold
        OCCDelay                      = 0x9281,   //Protections:OCC:Delay
        OCCRecoveryThreshold          = 0x9288,   //Protections:OCC:Recovery Threshold
        OCCPACKTOSDelta               = 0x92B0,   //Protections:OCC:PACK-TOS Delta
        OCD1Threshold                 = 0x9282,   //Protections:OCD1:Threshold
        OCD1Delay                     = 0x9283,   //Protections:OCD1:Delay
        OCD2Threshold                 = 0x9284,   //Protections:OCD2:Threshold
        OCD2Delay                     = 0x9285,   //Protections:OCD2:Delay
        SCDThreshold                  = 0x9286,   //Protections:SCD:Threshold
        SCDDelay                      = 0x9287,   //Protections:SCD:Delay
        SCDRecoveryTime               = 0x9294,   //Protections:SCD:Recovery Time
        OCD3Threshold                 = 0x928A,   //Protections:OCD3:Threshold
        OCD3Delay                     = 0x928C,   //Protections:OCD3:Delay
        OCDRecoveryThreshold          = 0x928D,   //Protections:OCD:Recovery Threshold
        OCDLLatchLimit                = 0x928F,   //Protections:OCDL:Latch Limit
        OCDLCounterDecDelay           = 0x9290,   //Protections:OCDL:Counter Dec Delay
        OCDLRecoveryTime              = 0x9291,   //Protections:OCDL:Recovery Time
        OCDLRecoveryThreshold         = 0x9292,   //Protections:OCDL:Recovery Threshold
        SCDLLatchLimit                = 0x9295,   //Protections:SCDL:Latch Limit
        SCDLCounterDecDelay           = 0x9296,   //Protections:SCDL:Counter Dec Delay
        SCDLRecoveryTime              = 0x9297,   //Protections:SCDL:Recovery Time
        SCDLRecoveryThreshold         = 0x9298,   //Protections:SCDL:Recovery Threshold
        OTCThreshold                  = 0x929A,   //Protections:OTC:Threshold
        OTCDelay                      = 0x920B,   //Protections:OTC:Delay
        OTCRecovery                   = 0x929C,   //Protections:OTC:Recovery
        OTDThreshold                  = 0x929D,   //Protections:OTD:Threshold
        OTDDelay                      = 0x929E,   //Protections:OTD:Delay
        OTDRecovery                   = 0x929F,   //Protections:OTD:Recovery
        OTFThreshold                  = 0x92A0,   //Protections:OTF:Threshold
        OTFDelay                      = 0x92A1,   //Protections:OTF:Delay
        OTFRecovery                   = 0x92A2,   //Protections:OTF:Recovery
        OTINTThreshold                = 0x92A3,   //Protections:OTINT:Threshold
        OTINTDelay                    = 0x92A4,   //Protections:OTINT:Delay
        OTINTRecovery                 = 0x92A5,   //Protections:OTINT:Recovery
        UTCThreshold                  = 0x92A6,   //Protections:UTC:Threshold
        UTCDelay                      = 0x92A7,   //Protections:UTC:Delay
        UTCRecovery                   = 0x92A8,   //Protections:UTC:Recovery
        UTDThreshold                  = 0x92A9,   //Protections:UTD:Threshold
        UTDDelay                      = 0x92AA,   //Protections:UTD:Delay
        UTDRecovery                   = 0x92AB,   //Protections:UTD:Recovery
        UTINTThreshold                = 0x92AC,   //Protections:UTINT:Threshold
        UTINTDelay                    = 0x92AD,   //Protections:UTINT:Delay
        UTINTRecovery                 = 0x92AE,   //Protections:UTINT:Recovery
        ProtectionsRecoveryTime       = 0x92AF,   //Protections:Recovery:Time
        HWDDelay                      = 0x92B2,   //Protections:HWD:Delay
        LoadDetectActiveTime          = 0x92B4,   //Protections:Load Detect:Active Time
        LoadDetectRetryDelay          = 0x92B5,   //Protections:Load Detect:Retry Delay
        LoadDetectTimeout             = 0x92B6,   //Protections:Load Detect:Timeout
        PTOChargeThreshold            = 0x92BA,   //Protections:PTO:Charge Threshold
        PTODelay                      = 0x92BC,   //Protections:PTO:Delay
        PTOReset                      = 0x92BE,   //Protections:PTO:Reset
        CUDEPThreshold                = 0x92C8,   //Permanent Fail:CUDEP:Threshold
        CUDEPDelay                    = 0x92CA,   //Permanent Fail:CUDEP:Delay
        SUVThreshold                  = 0x92CB,   //Permanent Fail:SUV:Threshold
        SUVDelay                      = 0x92CD,   //Permanent Fail:SUV:Delay
        SOVThreshold                  = 0x92CE,   //Permanent Fail:SOV:Threshold
        SOVDelay                      = 0x92D0,   //Permanent Fail:SOV:Delay
        TOSSThreshold                 = 0x92D1,   //Permanent Fail:TOS:Threshold
        TOSSDelay                     = 0x92D3,   //Permanent Fail:TOS:Delay
        SOCCThreshold                 = 0x92D4,   //Permanent Fail:SOCC:Threshold
        SOCCDelay                     = 0x92D6,   //Permanent Fail:SOCC:Delay
        SOCDThreshold                 = 0x92D7,   //Permanent Fail:SOCD:Threshold
        SOCDDelay                     = 0x92D9,   //Permanent Fail:SOCD:Delay
        SOTThreshold                  = 0x92DA,   //Permanent Fail:SOT:Threshold
        SOTDelay                      = 0x92DB,   //Permanent Fail:SOT:Delay
        SOTFThreshold                 = 0x92DC,   //Permanent Fail:SOTF:Threshold
        SOTFDelay                     = 0x92DD,   //Permanent Fail:SOTF:Delay
        VIMRCheckVoltage              = 0x92DE,   //Permanent Fail:VIMR:Check Voltage
        VIMRMaxRelaxCurrent           = 0x92E0,   //Permanent Fail:VIMR:Max Relax Current
        VIMRThreshold                 = 0x92E2,   //Permanent Fail:VIMR:Threshold
        VIMRDelay                     = 0x92E4,   //Permanent Fail:VIMR:Delay
        VIMRRelaxMinDuration          = 0x92E5,   //Permanent Fail:VIMR:Relax Min Duration
        VIMACheckVoltage              = 0x92E7,   //Permanent Fail:VIMA:Check Voltage
        VIMAMinActiveCurrent          = 0x92E9,   //Permanent Fail:VIMA:Min Active Current
        VIMAThreshold                 = 0x92EB,   //Permanent Fail:VIMA:Threshold
        VIMADelay                     = 0x92ED,   //Permanent Fail:VIMA:Delay
        CFETFOFFThreshold             = 0x92EE,   //Permanent Fail:CFETF:OFF Threshold
        CFETFOFFDelay                 = 0x92F0,   //Permanent Fail:CFETF:OFF Delay
        DFETFOFFThreshold             = 0x92F1,   //Permanent Fail:DFETF:OFF Threshold
        DFETFOFFDelay                 = 0x92F3,   //Permanent Fail:DFETF:OFF Delay
        VSSFFailThreshold             = 0x92F4,   //Permanent Fail:VSSF:Fail Threshold
        VSSFDelay                     = 0x92F6,   //Permanent Fail:VSSF:Delay
        PF2LVLDelay                   = 0x92F7,   //Permanent Fail:2LVL:Delay
        LFOFDelay                     = 0x92F8,   //Permanent Fail:LFOF:Delay
        HWMXDelay                     = 0x92F9,   //Permanent Fail:HWMX:Delay
        SecuritySettings              = 0x9256,   //Security:Settings:Security Settings
        UnsealKeyStep1                = 0x9257,   //Security:Keys:Unseal Key Step 1
        UnsealKeyStep2                = 0x9259,   //Security:Keys:Unseal Key Step 2
        FullAccessKeyStep1            = 0x925B,   //Security:Keys:Full Access Key Step 1
        FullAccessKeyStep2            = 0x925D,   //Security:Keys:Full Access Key Step 2
    };

    /** Direct Commands */
    enum CmdDrt : uint8_t {
        ControlStatus       = 0x00,
        SafetyAlertA        = 0x02,
        SafetyStatusA       = 0x03,
        SafetyAlertB        = 0x04,
        SafetyStatusB       = 0x05,
        SafetyAlertC        = 0x06,
        SafetyStatusC       = 0x07,
        PFAlertA            = 0x0A,
        PFStatusA           = 0x0B,
        PFAlertB            = 0x0C,
        PFStatusB           = 0x0D,
        PFAlertC            = 0x0E,
        PFStatusC           = 0x0F,
        PFAlertD            = 0x10,
        PFStatusD           = 0x11,
        BatteryStatus       = 0x12,
        Cell1Voltage        = 0x14,
        Cell2Voltage        = 0x16,
        Cell3Voltage        = 0x18,
        Cell4Voltage        = 0x1A,
        Cell5Voltage        = 0x1C,
        Cell6Voltage        = 0x1E,
        Cell7Voltage        = 0x20,
        Cell8Voltage        = 0x22,
        Cell9Voltage        = 0x24,
        Cell10Voltage       = 0x26,
        Cell11Voltage       = 0x28,
        Cell12Voltage       = 0x2A,
        Cell13Voltage       = 0x2C,
        Cell14Voltage       = 0x2E,
        Cell15Voltage       = 0x30,
        Cell16Voltage       = 0x32,
        StackVoltage        = 0x34,
        PACKPinVoltage      = 0x36,
        LDPinVoltage        = 0x38,
        CC2Current          = 0x3A,
        AlarmStatus         = 0x62,
        AlarmRawStatus      = 0x64,
        AlarmEnable         = 0x66,
        IntTemperature      = 0x68, // Internal DIE Temperature
        CFETOFFTemperature  = 0x6A,
        DFETOFFTemperature  = 0x6C,
        ALERTTemperature    = 0x6E,
        TS1Temperature      = 0x70,
        TS2Temperature      = 0x72,
        TS3Temperature      = 0x74,
        HDQTemperature      = 0x76,
        DCHGTemperature     = 0x78,
        DDSGTemperature     = 0x7A,
        FETStatus           = 0x7F,
    };

    /** Commands */
    enum Cmd : uint16_t {
        DEVICE_NUMBER           = 0x0001,
        FW_VERSION              = 0x0002,
        HW_VERSION              = 0x0003,
        IROM_SIG                = 0x0004,
        STATIC_CFG_SIG          = 0x0005,
        PREV_MACWRITE           = 0x0007,
        DROM_SIG                = 0x0009,
        SECURITY_KEYS           = 0x0035,
        SAVED_PF_STATUS         = 0x0053,
        MANUFACTURINGSTATUS     = 0x0057,
        MANU_DATA               = 0x0070,
        DASTATUS1               = 0x0071,
        DASTATUS2               = 0x0072,
        DASTATUS3               = 0x0073,
        DASTATUS4               = 0x0074,
        DASTATUS5               = 0x0075,
        DASTATUS6               = 0x0076,
        DASTATUS7               = 0x0077,
        CUV_SNAPSHOT            = 0x0080,
        COV_SNAPSHOT            = 0X0081,
        CB_ACTIVE_CELLS         = 0x0083,
        CB_SET_LVL              = 0x0084,
        CBSTATUS1               = 0x0085,
        CBSTATUS2               = 0x0086,
        CBSTATUS3               = 0x0087,
        FET_CONTROL             = 0x0097,
        REG12_CONTROL           = 0x0098,
        OTP_WR_CHECK            = 0x00A0,
        OTP_WRITE               = 0x00A1,
        READ_CAL1               = 0xF081,
        CAL_CUV                 = 0xF090,
        CAL_COV                 = 0xF091,

        /** Command Only Subcommands */
        EXIT_DEEPSLEEP       = 0x000E,
        DEEPSLEEP            = 0x000F,
        SHUTDOWN             = 0x0010,
        BQ769x2_RESET        = 0x0012,  //"RESET" in documentation
        PDSGTEST             = 0x001C,
        FUSE_TOGGLE          = 0x001D,
        PCHGTEST             = 0x001E,
        CHGTEST              = 0x001F,
        DSGTEST              = 0x0020,
        FET_ENABLE           = 0x0022,
        PF_ENABLE            = 0x0024,
        PF_RESET             = 0x0029,
        SEAL                 = 0x0030,
        RESET_PASSQ          = 0x0082,
        PTO_RECOVER          = 0x008A,
        SET_CFGUPDATE        = 0x0090,
        EXIT_CFGUPDATE       = 0x0092,
        DSG_PDSG_OFF         = 0x0093,
        CHG_PCHG_OFF         = 0x0094,
        ALL_FETS_OFF         = 0x0095,
        ALL_FETS_ON          = 0x0096,
        SLEEP_ENABLE         = 0x0099,
        SLEEP_DISABLE        = 0x009A,
        OCDL_RECOVER         = 0x009B,
        SCDL_RECOVER         = 0x009C,
        LOAD_DETECT_RESTART  = 0x009D,
        LOAD_DETECT_ON       = 0x009E,
        LOAD_DETECT_OFF      = 0x009F,
        CFETOFF_LO           = 0x2800,
        DFETOFF_LO           = 0x2801,
        ALERT_LO             = 0x2802,
        HDQ_LO               = 0x2806,
        DCHG_LO              = 0x2807,
        DDSG_LO              = 0x2808,
        CFETOFF_HI           = 0x2810,
        DFETOFF_HI           = 0x2811,
        ALERT_HI             = 0x2812,
        HDQ_HI               = 0x2816,
        DCHG_HI              = 0x2817,
        DDSG_HI              = 0x2818,
        PF_FORCE_A           = 0x2857,
        PF_FORCE_B           = 0x29A3,
        SWAP_COMM_MODE       = 0x29BC,
        SWAP_TO_I2C          = 0x29E7,
        SWAP_TO_SPI          = 0x7C35,
        SWAP_TO_HDQ          = 0x7C40,
        MCR_REG              = 0x1228,
    };

    /** returns false if timed */
    bool sendSubcommandR(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, Cmd cmd, uint8_t readOut[32]);

    /** returns false if timed */
    bool sendSubcommandW(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, Cmd cmd, uint8_t data);

    /** returns false if timed */
    bool sendSubcommandW2(System::I2C::I2C &i2c_controller,uint8_t i2c_addr, Cmd cmd, uint16_t data);

    /** returns false if timed */
    bool sendCommandSubcommand(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, Cmd command);

    /** returns false if timed */
    bool sendDirectCommandR(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, CmdDrt command, uint16_t * readOut);

    /** returns false if timed */
    bool sendDirectCommandW(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, CmdDrt command, uint16_t data);

    /** returns false if timed */
    bool setRegister(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, RegAddr reg_addr, uint32_t reg_data, uint8_t datalen);

    /** returns false if timed out */
    bool I2C_WriteReg(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

    /** returns false if timed */
    bool I2C_ReadReg(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count);

    bool spi24b_writeReg(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, uint8_t reg, uint8_t data);
//    void spi24b_readReg(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, uint8_t reg, uint8_t data);

    uint8_t Checksum(uint8_t *ptr, uint8_t len);

    uint8_t CRC8(uint8_t *ptr, uint8_t len);

};

#endif /* BQ769X2_PROTOCOL_HPP_ */
