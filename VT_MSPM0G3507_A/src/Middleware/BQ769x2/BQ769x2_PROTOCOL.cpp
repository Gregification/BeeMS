/*
 * baselined from TI's sample code found in the MSPM0 SDK.
 *      mspm0_sdk_2_05_00_05\examples\nortos\LP_MSPM0G3507\battery_gauge\gauge_level2_bq76952\Driver
 */

#include "Middleware/BQ769x2/BQ769x2_PROTOCOL.hpp"

#include <ti/driverlib/driverlib.h>

//******************************************************************************
// BQ Parameters ***************************************************************
//******************************************************************************

uint8_t BQ769X2_PROTOCOL::Checksum(uint8_t *ptr, uint8_t len)
// Calculates the checksum when writing to a RAM register. The checksum is the inverse of the sum of the bytes.
{
    uint8_t i;
    uint8_t checksum = 0;

    for (i = 0; i < len; i++) checksum += ptr[i];

    checksum = 0xff & ~checksum;

    return (checksum);
}

uint8_t BQ769X2_PROTOCOL::CRC8(uint8_t *ptr, uint8_t len)
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

bool BQ769X2_PROTOCOL::sendDirectCommandR(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, CmdDrt cmd, void * data_out, uint8_t datalen){

    for(uint8_t i = 0; i < datalen; i++)
        if(!spi24b_readReg(spi, cs, cmd+i, &((uint8_t*)data_out)[i]))
            return false;

    return true;
}

bool BQ769X2_PROTOCOL::sendDirectCommandW(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, CmdDrt cmd, void * data, uint8_t datalen) {
    for(uint8_t i = 0; i < datalen; i++)
        if(!spi24b_writeReg(spi, cs, cmd+i, ((uint8_t*)data)[i]))
            return false;

    return false;
}

bool BQ769X2_PROTOCOL::sendCommandSubcommand(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, Cmd cmd)  //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
// All that this function do is formatting the transfer array then writing the array to hex 3E,
// the monitor will then operate based on the command.
{  //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

    if(spi24b_writeReg(spi, cs, 0x3E, cmd & 0xFF))
    if(spi24b_writeReg(spi, cs, 0x3F, (cmd >> 8) & 0xFF))
        return true;

    return false;
}

bool BQ769X2_PROTOCOL::sendSubcommandR(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, Cmd cmd, void * data_out, uint8_t datalen) {
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    return readRegister(spi, cs, cmd, data_out, datalen);
}

bool BQ769X2_PROTOCOL::sendSubcommandW(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, Cmd cmd, uint8_t data) {
    //security keys and Manu_data writes dont work with this function (reading these commands works)
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot

    return true;
}

bool BQ769X2_PROTOCOL::sendSubcommandW2(System::I2C::I2C &i2c_controller,uint8_t i2c_addr, Cmd cmd, uint16_t data) {
    //security keys and Manu_data writes dont work with this function (reading these commands works)
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot

    return true;
}

bool BQ769X2_PROTOCOL::setRegister(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, RegAddr reg_addr, uint16_t reg_data, uint8_t datalen)
// This function will write hex 3E for the initial write for subcommands in direct memory
// and write to register hex 60 for the checksum to enter the data transmitted was correct.
// and there are different cases for the three varying data lengths.
{
    // BQTM.13.1/123

    // lower byte of address to 0x3E
    if(!spi24b_writeReg(spi, cs, 0x3E, reg_addr & 0xFF))
        return false;

    // upper byte of address to 0x3F
    if(!spi24b_writeReg(spi, cs, 0x3F, (reg_addr >> 8) & 0xFF))
        return false;

    // memory value in little endian format into the transfer buffer (0x40 to 0x5F)
    if(datalen > (0x5F - 0x40))
        datalen = 0x5F - 0x40;
    for(uint8_t i = 0; i < datalen; i++){
        if(!spi24b_writeReg(spi, cs,
                    0x40 + i,
                    ((uint8_t *)&reg_data)[i]
                ))
            return false;
    }

    // checksum of data written goes in 0x60
    { // checksum is {addr low B, addr high B, <data ... >}
        // max data is 32B
        uint16_t crcRange[] = {reg_addr, reg_data};

        if(!spi24b_writeReg(spi, cs, 0x60, Checksum((uint8_t*)crcRange, sizeof(reg_addr) + datalen)))
            return false;
    }

    // data len in 0x61
    if(!spi24b_writeReg(spi, cs, 0x61, datalen + 4)) // +4 : datalen + sizeof({0x3E, 0x3F, 0x60, 0x61})
        return false;

    return true;
}

uint8_t BQ769X2_PROTOCOL::readRegister(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, uint16_t reg_addr, void * data_out, uint8_t datalen){
    // BQTM.13.1/123

    // lower byte of address to 0x3E
    if(!spi24b_writeReg(spi, cs, 0x3E, reg_addr & 0xFF))
        return false;

    // upper byte of address to 0x3F
    if(!spi24b_writeReg(spi, cs, 0x3F, (reg_addr >> 8) & 0xFF))
        return false;

    // memory value in little endian format out of the transfer buffer (0x40 to 0x5F)
    uint8_t datalen_r; // response length
    if(!spi24b_readReg(spi, cs, 0x61, &datalen))
        return false;
    if(datalen_r < datalen)
        datalen = datalen_r;

    for(uint8_t i = 0; i < datalen && (0x40+i) <= 0x5F; i++){
        if(!spi24b_readReg(spi, cs,
                    0x40 + i,
                    &((uint8_t *)data_out)[i]
                ))
            return false;
    }

    // checksum of entire data set is in 0x60
    // i dont bother checking since each byte is CRC checked when its read

    return datalen;
}

bool BQ769X2_PROTOCOL::spi24b_writeReg(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, uint8_t reg, uint8_t data){
    if(!spi) return false;

    uint8_t tx[3], rx[3] = {0,0,0};
    uint8_t retries;
    constexpr int max_retries = 5;

    tx[0] = BV(7) | reg;
    tx[1] = data;
    tx[2] = CRC8(tx, 2);

    retries = 0;
    do{
        if(retries != 0)
            vTaskDelay(pdMS_TO_TICKS(retries));

        spi->transfer_blocking(tx, rx, 3, cs);

        if(rx[0] == tx[0])
        if(rx[1] == tx[1])
        if(rx[2] == tx[2])
            return true;
    } while(retries++ < max_retries);

    // TODO: duplicate transmissions on some start up cases. see oscilloscope.
    //      not a pressing issue, just might waste a few mS here and there.

    return false;
}

bool BQ769X2_PROTOCOL::spi24b_readReg(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, uint8_t reg, uint8_t * data_out){
    if(!spi) return false;

    uint8_t tx[3], rx[3];
    uint8_t retries;
    constexpr int max_retries = 6;

    tx[0] = reg & ~BV(7); //7th bit = 0 for read
    tx[1] = 0; // arbitrary. only used for CRC
    tx[2] = CRC8(tx, 2);

    retries = 0;
    do{
        if(retries != 0)
            vTaskDelay(pdMS_TO_TICKS(retries));

        spi->transfer_blocking(tx, rx, 3, cs);


//        if(rx[0] == 0xFF && rx[1] == 0xFF){
//            switch(rx[2]){
//                case 0x00: // 0xFF'FF'00 : prev transaction incomplete.
//                case 0xAA: // 0xFF'FF'AA : former CRC wrong
//                case 0xFF: // 0xFF'FF'FF : internal clock not powered.
//                    continue;
//                default:
//            }
//        }

        if(rx[0] == tx[0]) { // yippie!
            // check CRC
            if(rx[2] != CRC8(rx,2))
                continue;

            *data_out = rx[1];
            return true;
        }

    } while(retries++ < max_retries);

    return false;
}

//************************************End of BQ769X2_PROTOCOL Measurement Commands******************************************
