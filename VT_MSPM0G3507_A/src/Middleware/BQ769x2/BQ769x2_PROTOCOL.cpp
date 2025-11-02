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

bool BQ769X2_PROTOCOL::sendDirectCommandR(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, CmdDrt command, uint16_t * readOut){
    uint8_t buff[4];

//    I2C_ReadReg(i2c_controller, i2c_addr, command, buff, sizeof(buff));

    *readOut = buff[2];
    *readOut <<= 8;
    *readOut |= buff[0];
    return true;
}

bool BQ769X2_PROTOCOL::sendDirectCommandW(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, CmdDrt command, uint16_t data) {
    uint8_t TX_data[2] = {0x00, 0x00};

    //little endian format
    TX_data[0] = data & 0xff;
    TX_data[1] = (data >> 8) & 0xff;

    //Control_status, alarm_status, alarm_enable all 2 bytes long
//    if(I2C_WriteReg(i2c_controller, i2c_addr, command, TX_data, 2))
//        return true;
//    else
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

bool BQ769X2_PROTOCOL::sendSubcommandR(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, Cmd cmd, uint8_t readOut[32]) {
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot

    return true;
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

bool BQ769X2_PROTOCOL::setRegister(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, RegAddr reg_addr, uint32_t reg_data, uint8_t datalen)
// This function will write hex 3E for the initial write for subcommands in direct memory
// and write to register hex 60 for the checksum to enter the data transmitted was correct.
// and there are different cases for the three varying data lengths.
{
    // BQTM.13.1/123
    volatile uint8_t txB;// debugging

    // lower byte of address to 0x3E
    txB = reg_addr & 0xFF;
    if(!spi24b_writeReg(spi, cs, 0x3E, reg_addr & 0xFF))
        return false;

    // upper byte of address to 0x3F
    txB = (reg_addr >> 8) & 0xFF;
    if(!spi24b_writeReg(spi, cs, 0x3F, (reg_addr >> 8) & 0xFF))
        return false;

    // memory value in little endian format into the transfer buffer (0x40 to 0x5F)
    for(uint8_t i = 0; i < datalen; i++){
        txB = ((uint8_t *)&reg_data)[i];
        if(!spi24b_writeReg(spi, cs,
                    0x40 + i,
                    ((uint8_t *)&reg_data)[i]
                ))
            return false;
    }

    // checksum of data written goes in 0x60
    { // checksum is {addr low B, addr high B, <data ... >}
        // max data is 32B
        uint8_t crcRange[32 + 2];
        ((uint16_t*)crcRange)[0] = reg_addr;
        uint8_t i;
        for(i = 0; i < datalen && i < sizeof(crcRange - 2) && i < sizeof(reg_data); i++) {
            crcRange[2 + i] = ((uint8_t *)&reg_data)[i];
        }

        txB = Checksum(crcRange, i+2);
        if(!spi24b_writeReg(spi, cs, 0x60, Checksum(crcRange, i+2)))
            return false;
    }

    // data len in 0x61
    txB = datalen + 4;
    if(!spi24b_writeReg(spi, cs, 0x61, datalen + 4)) // +4 : datalen + sizeof({0x3E, 0x3F, 0x60, 0x61})
        return false;

    return true;
}

bool BQ769X2_PROTOCOL::spi24b_writeReg(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, uint8_t reg, uint8_t data){
    if(!spi) return false;

    uint8_t tx[3], rx[3];
    uint8_t retries;
    constexpr int max_retries = 5;

    tx[0] = BV(7) | reg;
    tx[1] = data;
    tx[2] = CRC8(&tx[0], 2);

    retries = 0;
    do{
        spi->transfer_blocking(tx, rx, 3, cs);

        retries++;
        if(retries != 1)
            vTaskDelay(pdMS_TO_TICKS(retries));
    } while(
                (retries < max_retries)
            &&  (rx[0] == 0xFF)
            &&  (rx[1] == 0xFF)
            &&  (
                        (rx[2] == 0x00) // 0xFF'FF'00 : prev transaction incomplete.
                    ||  (rx[2] == 0xAA) // 0xFF'FF'AA : former CRC wrong
                    ||  (rx[2] == 0xFF) // 0xFF'FF'FF : internal clock not powered.
                )
        );

    // TODO: duplicate transmissions on some start up cases. see oscilloscope.
    //      not a pressing issue, just might waste a few mS here and there.

    return      (retries <= max_retries)
            && !(
                    (rx[0] == 0xFF)
                &&  (rx[1] == 0xFF)
                &&  (
                        (rx[2] == 0xFF) // 0xFF'FF'FF : internal clock not powered.
                    ||  (rx[2] == 0x00) // 0xFF'FF'00 : prev transaction incomplete.
                    )
                )
        ;
}

//************************************End of BQ769X2_PROTOCOL Measurement Commands******************************************
