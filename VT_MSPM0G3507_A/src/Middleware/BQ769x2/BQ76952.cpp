/*
 * BQ76952.cpp
 *
 *  Created on: Aug 13, 2025
 *      Author: FSAE
 */

#include "BQ76952.hpp"
#include "Middleware/BQ769x2/BQ769x2_PROTOCOL.hpp"

bool BQ76952::sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd command) {
    return BQ769X2_PROTOCOL::sendCommandSubcommand(spi, cs, command);
}

bool BQ76952::sendSubcommandR(BQ769X2_PROTOCOL::Cmd cmd, void * data_out, uint8_t datalen) {
    return BQ769X2_PROTOCOL::sendSubcommandR(spi, cs, cmd, data_out, datalen);
}

bool BQ76952::sendSubcommandW(BQ769X2_PROTOCOL::Cmd cmd, uint16_t data, uint8_t datalen) {
    return BQ769X2_PROTOCOL::sendSubcommandW(spi, cs, cmd, data, datalen);
//    return BQ769X2_PROTOCOL::sendSubcommandW(*i2c_controller, i2c_addr, cmd, data);
}

bool BQ76952::sendCommandR(BQ769X2_PROTOCOL::Cmd command, void * data_out, uint8_t datalen){
    return BQ769X2_PROTOCOL::readRegister(spi, cs, command, data_out, datalen);
}

bool BQ76952::sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt command, void * readOut, uint8_t datalen) {
    return BQ769X2_PROTOCOL::sendDirectCommandR(spi, cs, command, readOut, datalen);
}

bool BQ76952::sendDirectCommandW(BQ769X2_PROTOCOL::CmdDrt command, void * data, uint8_t datalen) {
    return BQ769X2_PROTOCOL::sendDirectCommandW(spi, cs, command, data, datalen);
}

bool BQ76952::setRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, uint16_t reg_data, uint8_t datalen) {
    return BQ769X2_PROTOCOL::setRegister(spi, cs, reg_addr, reg_data, datalen);
}

uint8_t BQ76952::getRegister(BQ769X2_PROTOCOL::RegAddr reg_addr, void * reg_out, uint8_t datalen){
    return BQ769X2_PROTOCOL::readRegister(spi, cs, reg_addr, reg_out, datalen);
}

bool BQ76952::unseal(uint32_t key){
    // SEAL -> UNSEAL
    //bqTM.13.8.2/197
    /* bqTM.8.1/71
     *  """
     *      each transition requires that a unique set of keys be sent to the device
     *      through the sub-command address (0x3E and 0x3F). The keys must be sent
     *      consecutively to 0x3E and 0x3F, with no other data written between the
     *      keys. Do not set the two keys to identical values
     *  """
     */

    if(setRegister(BQ769X2_PROTOCOL::RegAddr::SECURITY_KEY_ENTRY_1, ARRANDN(((uint8_t*)&key)[3])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::SECURITY_KEY_ENTRY_2, ARRANDN(((uint8_t*)&key)[2])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::SECURITY_KEY_ENTRY_1, ARRANDN(((uint8_t*)&key)[1])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::SECURITY_KEY_ENTRY_2, ARRANDN(((uint8_t*)&key)[0])))
        return true;

    return false;
}

// TODO: complete the function
/*bool BQ76952::getConfig(BQ76952SSetting * config){
    bool success = false;

    if(getRegister(BQ769X2_PROTOCOL::RegAddr::PowerConfig, PTRANDN(config->Configuration.powerConfig.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::REG0Config, PTRANDN(config->Configuration.REG0Config.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::HWDRegulatorOptions, PTRANDN(config->Configuration.HWDRegulatorOptions.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::CommIdleTime, PTRANDN(config->Configuration.commIdleTime_S)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::CFETOFFPinConfig, PTRANDN(config->Configuration.cfetoffPinConfig.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::DFETOFFPinConfig, PTRANDN(config->Configuration.dfetoffPinConfig.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::ALERTPinConfig, PTRANDN(config->Configuration.alertPinConfig.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::TS1Config, PTRANDN(config->Configuration.TS1Config.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::TS2Config, PTRANDN(config->Configuration.TS2Config.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::TS3Config, PTRANDN(config->Configuration.TS3Config.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::HDQPinConfig, PTRANDN(config->Configuration.HDQPinConfig.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::DCHGPinConfig, PTRANDN(config->Configuration.DCHGPinConfig.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::DDSGPinConfig, PTRANDN(config->Configuration.DDSGPinConfig.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::DAConfiguration, PTRANDN(config->Configuration.DAConfig.Raw)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::VCellMode, PTRANDN(config->Configuration.VcellMode)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::CC3Samples, PTRANDN(config->Configuration.CC3Samples)))
    if(getRegister(BQ769X2_PROTOCOL::RegAddr::DAConfiguration, PTRANDN(config->Configuration.DAConfig.Raw)))
        success = true;
    //TODO: finish adding all registers

    return success;
}*/

bool BQ76952::setConfig(BQ76952SSetting const * config){
    bool success = false;

    if(!sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::SET_CFGUPDATE))
        return false;
    vTaskDelay(pdMS_TO_TICKS(8));

    if(setRegister(BQ769X2_PROTOCOL::RegAddr::MinBlowFuseVoltage, ARRANDN(config->Fuse.minBlowFuseVoltage_10mV)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::FuseBlowTimeout, ARRANDN(config->Fuse.timeout_S)))

    if(setRegister(BQ769X2_PROTOCOL::RegAddr::PowerConfig, ARRANDN(config->Configuration.powerConfig.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::REG12Config, ARRANDN(config->Configuration.REG12Config.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::REG0Config, ARRANDN(config->Configuration.REG0Config.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::HWDRegulatorOptions, ARRANDN(config->Configuration.HWDRegulatorOptions.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::SPIConfiguration, ARRANDN(config->Configuration.spiConfig.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CommIdleTime, ARRANDN(config->Configuration.commIdleTime_S)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CFETOFFPinConfig, ARRANDN(config->Configuration.cfetoffPinConfig.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::DFETOFFPinConfig, ARRANDN(config->Configuration.dfetoffPinConfig.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::ALERTPinConfig, ARRANDN(config->Configuration.alertPinConfig.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::TS1Config, ARRANDN(config->Configuration.TS1Config.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::TS2Config, ARRANDN(config->Configuration.TS2Config.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::TS3Config, ARRANDN(config->Configuration.TS3Config.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::HDQPinConfig, ARRANDN(config->Configuration.HDQPinConfig.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::DCHGPinConfig, ARRANDN(config->Configuration.DCHGPinConfig.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::DDSGPinConfig, ARRANDN(config->Configuration.DDSGPinConfig.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::DAConfiguration, ARRANDN(config->Configuration.DAConfig.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::VCellMode, ARRANDN(config->Configuration.VcellMode)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CC3Samples, ARRANDN(config->Configuration.CC3Samples)))

    if(setRegister(BQ769X2_PROTOCOL::RegAddr::ProtectionConfiguration, ARRANDN(config->Protection.protectionConfiguraiton.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledProtectionsA, ARRANDN(config->Protection.enabledProtectionsA.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledProtectionsB, ARRANDN(config->Protection.enabledProtectionsB.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledProtectionsC, ARRANDN(config->Protection.enabledProtectionsC.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CHGFETProtectionsA, ARRANDN(config->Protection.chgFetProtectionsA.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CHGFETProtectionsB, ARRANDN(config->Protection.chgFetProtectionsB.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CHGFETProtectionsC, ARRANDN(config->Protection.chgFetProtectionsC.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::DSGFETProtectionsA, ARRANDN(config->Protection.dsgFetProtectionsA.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::DSGFETProtectionsB, ARRANDN(config->Protection.dsgFetProtectionsB.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::DSGFETProtectionsC, ARRANDN(config->Protection.dsgFetProtectionsC.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::BodyDiodeThreshold, ARRANDN(config->Protection.bodyDiodeThreshold)))

    if(setRegister(BQ769X2_PROTOCOL::RegAddr::DefaultAlarmMask, ARRANDN(config->Alarm.defaultAlarmMask.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::SFAlertMaskA, ARRANDN(config->Alarm.sfAlertMaskA.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::SFAlertMaskB, ARRANDN(config->Alarm.sfAlertMaskB.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::SFAlertMaskC, ARRANDN(config->Alarm.sfAlertMaskC.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::PFAlertMaskA, ARRANDN(config->Alarm.pfAlertMaskA.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::PFAlertMaskB, ARRANDN(config->Alarm.pfAlertMaskB.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::PFAlertMaskC, ARRANDN(config->Alarm.pfAlertMaskC.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::PFAlertMaskD, ARRANDN(config->Alarm.pfAlertMaskD.Raw)))

    if(setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledPFA, ARRANDN(config->PermanentFailure.enabledPFA.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledPFB, ARRANDN(config->PermanentFailure.enabledPFB.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledPFC, ARRANDN(config->PermanentFailure.enabledPFC.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledPFD, ARRANDN(config->PermanentFailure.enabledPFD.Raw)))

    if(setRegister(BQ769X2_PROTOCOL::RegAddr::FETOptions, ARRANDN(config->FET.fetOptions.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::ChgPumpControl, ARRANDN(config->FET.chgPumpControl.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::PrechargeStartVoltage, ARRANDN(config->FET.prechargeStartVoltage)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::PrechargeStopVoltage, ARRANDN(config->FET.prechargeStopVoltage)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::PredischargeStopDelta, ARRANDN(config->FET.predischargeStopDelta)))

    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CheckTime, ARRANDN(config->CellOpenWire.checkTime_S)))

    // be explicit
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell1Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[0])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell2Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[1])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell3Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[2])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell4Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[3])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell5Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[4])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell6Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[5])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell7Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[6])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell8Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[7])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell9Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[8])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell10Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[9])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell11Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[10])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell12Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[11])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell13Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[12])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell14Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[13])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell15Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[14])))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::Cell16Interconnect, ARRANDN(config->InterconnectResistance.cellInterconnectResistance_mOhm[15])))

    if(setRegister(BQ769X2_PROTOCOL::RegAddr::BalancingConfiguration, ARRANDN(config->CellBalancingConfig.balancingConfiguration.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::MinCellTemp, ARRANDN(config->CellBalancingConfig.minCellTemp_C)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::MaxCellTemp, ARRANDN(config->CellBalancingConfig.maxCellTemp_C)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::MaxInternalTemp, ARRANDN(config->CellBalancingConfig.maxInternalTemp_C)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceInterval, ARRANDN(config->CellBalancingConfig.cellBalanceInterval_s)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMaxCells, ARRANDN(config->CellBalancingConfig.cellBalanceMaxCells)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinCellVCharge, ARRANDN(config->CellBalancingConfig.cellBalanceMinCellV_Charge_mV)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinDeltaCharge, ARRANDN(config->CellBalancingConfig.cellBalanceMinDelta_Charge_mV)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceStopDeltaCharge, ARRANDN(config->CellBalancingConfig.cellBalanceStopDelta_Charge_mV)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinCellVRelax, ARRANDN(config->CellBalancingConfig.cellBalanceMinCellV_Relax_mV)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinDeltaRelax, ARRANDN(config->CellBalancingConfig.cellBalanceMinDelta_Relax_mV)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceStopDeltaRelax, ARRANDN(config->CellBalancingConfig.cellBalanceStopDelta_Relax_mV)))

        success = true;

    //TODO: finish adding all registers

    vTaskDelay(pdMS_TO_TICKS(9));
    if(!sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::EXIT_CFGUPDATE))
        success = false;
    vTaskDelay(pdMS_TO_TICKS(9));

    // clear alarms
    {
        uint16_t alarm;
        sendDirectCommandR(BQ769X2_PROTOCOL::CmdDrt::AlarmStatus, &alarm, sizeof(alarm));
        sendDirectCommandW(BQ769X2_PROTOCOL::CmdDrt::AlarmStatus, &alarm, sizeof(alarm));
    }

    return success;
}

bool BQ76952::BQ76952PinConfig::operator==(const BQ76952PinConfig& other) const{
    return Raw == other.Raw;
}

bool BQ76952::BQ76952SSetting::operator==(const BQ76952SSetting& other) const {

    // 1. Compare non-union members directly
    if (Fuse.minBlowFuseVoltage_10mV != other.Fuse.minBlowFuseVoltage_10mV) return false;
    if (Fuse.timeout_S != other.Fuse.timeout_S) return false;
    if (Configuration.commIdleTime_S != other.Configuration.commIdleTime_S) return false;
    if (Configuration.VcellMode != other.Configuration.VcellMode) return false;
    if (Configuration.CC3Samples != other.Configuration.CC3Samples) return false;

    // 2. Compare BQ76952PinConfig members using their own operator==
    if (!(Configuration.cfetoffPinConfig == other.Configuration.cfetoffPinConfig)) return false;
    if (!(Configuration.dfetoffPinConfig == other.Configuration.dfetoffPinConfig)) return false;
    if (!(Configuration.alertPinConfig == other.Configuration.alertPinConfig)) return false;
    if (!(Configuration.TS1Config == other.Configuration.TS1Config)) return false;
    if (!(Configuration.TS2Config == other.Configuration.TS2Config)) return false;
    if (!(Configuration.TS3Config == other.Configuration.TS3Config)) return false;
    if (!(Configuration.HDQPinConfig == other.Configuration.HDQPinConfig)) return false;
    if (!(Configuration.DCHGPinConfig == other.Configuration.DCHGPinConfig)) return false;
    if (!(Configuration.DDSGPinConfig == other.Configuration.DDSGPinConfig)) return false;

    // 3. Compare Union members with reserved bits using a MASK

    // PowerConfig (2 reserved bits: bits 14, 15) -> Mask 0x3FFF
    const uint16_t POWER_MASK = 0x3FFF;
    if ((Configuration.powerConfig.Raw & POWER_MASK) != (other.Configuration.powerConfig.Raw & POWER_MASK)) return false;

    // REG0Config (7 reserved bits: bits 1-7) -> Mask 0x01
    const uint8_t REG0_MASK = 0x01;
    if ((Configuration.REG0Config.Raw & REG0_MASK) != (other.Configuration.REG0Config.Raw & REG0_MASK)) return false;

    // HWDRegulatorOptions (2 reserved bits: bits 6, 7) -> Mask 0x3F
    const uint8_t HWD_MASK = 0x3F;
    if ((Configuration.HWDRegulatorOptions.Raw & HWD_MASK) != (other.Configuration.HWDRegulatorOptions.Raw & HWD_MASK)) return false;

    // spiConfig (5 reserved bits at start, 1 at end: bits 0-4, 7) -> Mask 0x60
    // (filt is bit 5, miso_reg1 is bit 6)
    const uint8_t SPI_MASK = 0x60;
    if ((Configuration.spiConfig.Raw & SPI_MASK) != (other.Configuration.spiConfig.Raw & SPI_MASK)) return false;

    // DAConfig (3 reserved bits: bits 5, 6, 7) -> Mask 0x1F
    const uint8_t DA_MASK = 0x1F;
    if ((Configuration.DAConfig.Raw & DA_MASK) != (other.Configuration.DAConfig.Raw & DA_MASK)) return false;

    // If all fields match (or match after masking reserved bits), the structs are equal
    return true;
}
