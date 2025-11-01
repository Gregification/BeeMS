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

    I2C_ReadReg(i2c_controller, i2c_addr, command, buff, sizeof(buff));

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
    if(I2C_WriteReg(i2c_controller, i2c_addr, command, TX_data, 2))
        return true;
    else
        return false;
}

bool BQ769X2_PROTOCOL::sendCommandSubcommand(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, Cmd cmd)  //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
// All that this function do is formatting the transfer array then writing the array to hex 3E,
// the monitor will then operate based on the command.
{  //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

    spi24b_writeReg(spi, cs, 0x3E, cmd & 0xFF);
    spi24b_writeReg(spi, cs, 0x3F, (cmd >> 8) & 0xFF);
    return true;
}

//bool BQ769X2_PROTOCOL::sendSubcommand(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, Cmd command, uint16_t data, DIR_CMD_TYPE type, TickType_t timeout)
//// See the TRM or the BQ76952 header file for a full list of Subcommands
//// The input type can either be the defined macros R for read, W for write, or W2 for write two bytes.
//{
//    //security keys and Manu_data writes dont work with this function (reading these commands works)
//    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
//    uint8_t TX_Reg[4]    = {0x00, 0x00, 0x00, 0x00};
//    uint8_t TX_Buffer[2] = {0x00, 0x00};
//
//    //TX_Reg in little endian format
//    TX_Reg[0] = command & 0xff;
//    TX_Reg[1] = (command >> 8) & 0xff;
//
//    if (type == BQ769X2_PROTOCOL::DIR_CMD_TYPE::R) {  //read
//        I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_Reg, 2);
//        delayUS(2000);
//        I2C_ReadReg(i2c_controller, i2c_addr, 0x40, RX_32Byte, 32);  //RX_32Byte is a global variable
//    } else if (type == BQ769X2_PROTOCOL::DIR_CMD_TYPE::W) {
//        //FET_Control, REG12_Control
//        TX_Reg[2] = data & 0xff;
//        I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_Reg, 3);
//        delayUS(1000);
//        TX_Buffer[0] = Checksum(TX_Reg, 3);
//        TX_Buffer[1] = 0x05;  //combined length of registers address and data
//        I2C_WriteReg(i2c_controller, i2c_addr, 0x60, TX_Buffer, 2);
//        delayUS(1000);
//    } else if (type == BQ769X2_PROTOCOL::DIR_CMD_TYPE::W2) {  //write data with 2 bytes
//        //CB_Active_Cells, CB_SET_LVL
//        TX_Reg[2] = data & 0xff;
//        TX_Reg[3] = (data >> 8) & 0xff;
//        I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_Reg, 4);
//        delayUS(1000);
//        TX_Buffer[0] = Checksum(TX_Reg, 4);
//        TX_Buffer[1] = 0x06;  //combined length of registers address and data
//        I2C_WriteReg(i2c_controller, i2c_addr, 0x60, TX_Buffer, 2);
//        delayUS(1000);
//    }
//}

bool BQ769X2_PROTOCOL::sendSubcommandR(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, Cmd cmd, uint8_t readOut[32]) {
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    uint8_t TX_Reg[4]    = {0x00, 0x00, 0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = cmd & 0xff;
    TX_Reg[1] = (cmd >> 8) & 0xff;

    if(!I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_Reg, 2))
        return false;

    vTaskDelay(pdMS_TO_TICKS(2));

    if(!I2C_ReadReg(i2c_controller, i2c_addr, 0x40, readOut, 32))
        return false;

    return true;
}

bool BQ769X2_PROTOCOL::sendSubcommandW(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, Cmd cmd, uint8_t data) {
    //security keys and Manu_data writes dont work with this function (reading these commands works)
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    uint8_t TX_Reg[4]    = {0x00, 0x00, 0x00, 0x00};
    uint8_t TX_Buffer[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = cmd & 0xff;
    TX_Reg[1] = (cmd >> 8) & 0xff;

    //FET_Control, REG12_Control
    TX_Reg[2] = data & 0xff;
    if(!I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_Reg, 3))
        return false;
    delay_cycles(System::CLK::CPUCLK/1000 * 1);
    TX_Buffer[0] = Checksum(TX_Reg, 3);
    TX_Buffer[1] = 0x05;  //combined length of registers address and data
    if(!I2C_WriteReg(i2c_controller, i2c_addr, 0x60, TX_Buffer, 2))
        return false;
    delay_cycles(System::CLK::CPUCLK/1000 * 1);

    return true;
}

bool BQ769X2_PROTOCOL::sendSubcommandW2(System::I2C::I2C &i2c_controller,uint8_t i2c_addr, Cmd cmd, uint16_t data) {
    //security keys and Manu_data writes dont work with this function (reading these commands works)
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    uint8_t TX_Reg[4]    = {0x00, 0x00, 0x00, 0x00};
    uint8_t TX_Buffer[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = cmd & 0xff;
    TX_Reg[1] = (cmd >> 8) & 0xff;

    //CB_Active_Cells, CB_SET_LVL
    TX_Reg[2] = data & 0xff;
    TX_Reg[3] = (data >> 8) & 0xff;
    if(!I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_Reg, 4))
        return false;
    delay_cycles(System::CLK::CPUCLK/1000 * 1);
    TX_Buffer[0] = Checksum(TX_Reg, 4);
    TX_Buffer[1] = 0x06;  //combined length of registers address and data
    if(!I2C_WriteReg(i2c_controller, i2c_addr, 0x60, TX_Buffer, 2))
        return false;
    delay_cycles(System::CLK::CPUCLK/1000 * 1);

    return true;
}

bool BQ769X2_PROTOCOL::setRegister(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, RegAddr reg_addr, uint32_t reg_data, uint8_t datalen)
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
            if(I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_RegData, 3))
                vTaskDelay(pdMS_TO_TICKS(2));
            else return false;

            TX_Buffer[0] = Checksum(TX_RegData, 3);
            TX_Buffer[1] = 0x05;  //combined length of register address and data
            if(I2C_WriteReg(i2c_controller, i2c_addr, 0x60, TX_Buffer, 2))  // Write the checksum and length
                vTaskDelay(pdMS_TO_TICKS(2));
            else return false;

            break;
        case 2:  //2 byte datalength
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            if(I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_RegData, 4))
                vTaskDelay(pdMS_TO_TICKS(2));
            else return false;

            TX_Buffer[0] = Checksum(TX_RegData, 4);
            TX_Buffer[1] = 0x06;  //combined length of register address and data
            if(I2C_WriteReg(i2c_controller, i2c_addr, 0x60, TX_Buffer, 2))  // Write the checksum and length
                vTaskDelay(pdMS_TO_TICKS(2));
            else return false;

            break;

        case 4:  //4 byte datalength, Only used for CCGain and Capacity Gain
            TX_RegData[3] = (reg_data >> 8) & 0xff;
            TX_RegData[4] = (reg_data >> 16) & 0xff;
            TX_RegData[5] = (reg_data >> 24) & 0xff;
            if(I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_RegData, 6))
                vTaskDelay(pdMS_TO_TICKS(2));
            else return false;

            TX_Buffer[0] = Checksum(TX_RegData, 6);
            TX_Buffer[1] = 0x08;  //combined length of register address and data
            if(I2C_WriteReg(i2c_controller, i2c_addr, 0x60, TX_Buffer, 2))  // Write the checksum and length
                vTaskDelay(pdMS_TO_TICKS(2));
            else return false;

            break;
    }

    return true;
}

bool BQ769X2_PROTOCOL::I2C_ReadReg(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    // tx read address then rx fetched
    do {
        if(! i2c_controller.tx_blocking(i2c_addr, &reg_addr, 1, pdMS_TO_TICKS(4)))
            break;

        vTaskDelay(pdMS_TO_TICKS(3));

        if(! i2c_controller.rx_blocking(i2c_addr, reg_data, count, pdMS_TO_TICKS(4)))
            break;

        return true;
    }while(false);

    return false;
}


bool BQ769X2_PROTOCOL::I2C_WriteReg(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    uint8_t I2Ctxbuff[8] = {0x00};

    I2Ctxbuff[0] = reg_addr;
    uint8_t i, j = 1;

    for (i = 0; i < count; i++) {
        I2Ctxbuff[j] = reg_data[i];
        j++;
    }

    return i2c_controller.tx_blocking(i2c_addr, I2Ctxbuff, count+1, pdMS_TO_TICKS(10));
}

bool BQ769X2_PROTOCOL::spi24b_writeReg(System::SPI::SPI * spi, System::GPIO::GPIO const * cs, uint8_t reg, uint8_t data){
    if(!spi) return false;

    static uint8_t tx[3], rx[3];
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
                        (rx[2] == 0xFF) // 0xFF'FF'FF : internal clock not powered.
                    ||  (rx[2] == 0x00) // 0xFF'FF'00 : prev transaction incomplete.
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
