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

bool BQ76952::sendSubcommandW(BQ769X2_PROTOCOL::Cmd cmd, uint8_t data) {
    System::FailHard("BQ76952::sendSubcommandW DNE");
    return false;
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

bool BQ76952::getConfig(BQ76952SSetting * config){
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
}

bool BQ76952::setConfig(BQ76952SSetting const * config){
    bool success = false;

    if(!sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::SET_CFGUPDATE))
        return false;
    vTaskDelay(pdMS_TO_TICKS(8));

    if(setRegister(BQ769X2_PROTOCOL::RegAddr::PowerConfig, ARRANDN(config->Configuration.powerConfig.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::REG0Config, ARRANDN(config->Configuration.REG0Config.Raw)))
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::HWDRegulatorOptions, ARRANDN(config->Configuration.HWDRegulatorOptions.Raw)))
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
    if(setRegister(BQ769X2_PROTOCOL::RegAddr::DAConfiguration, ARRANDN(config->Configuration.DAConfig.Raw)))
        success = true;
    //TODO: finish adding all registers

    vTaskDelay(pdMS_TO_TICKS(9));
    if(!sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::EXIT_CFGUPDATE))
        return false;
    vTaskDelay(pdMS_TO_TICKS(9));

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
