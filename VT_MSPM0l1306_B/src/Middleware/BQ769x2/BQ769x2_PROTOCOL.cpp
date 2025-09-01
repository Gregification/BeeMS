/*
 * baselined from TI's sample code found in the MSPM0 SDK.
 *      mspm0_sdk_2_05_00_05\examples\nortos\LP_MSPM0G3507\battery_gauge\gauge_level2_bq76952\Driver
 */

#include <FreeRTOS.h>
#include <task.h>

#include "Middleware/BQ769x2/BQ769x2_PROTOCOL.hpp"

//******************************************************************************
// BQ Parameters ***************************************************************
//******************************************************************************

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

//bool  BQ769X2_PROTOCOL::sendDirectCommand(System::I2C::I2C i2c_controller, uint8_t i2c_addr, CmdDrt command, uint16_t data, DIR_CMD_TYPE type, TickType_t timeout)
//{
//    uint8_t TX_data[2] = {0x00, 0x00};
//
//    //little endian format
//    TX_data[0] = data & 0xff;
//    TX_data[1] = (data >> 8) & 0xff;
//
//    uint16_t ret = 0;
//    if (type == R) {                       //Read
//        I2C_ReadReg(i2c_controller, i2c_addr, command, &ret, 2, timeout);  //RX_data is a global variable
//        vTaskDelay(0);
//    }
//    if (type == W) {  //write
//        //Control_status, alarm_status, alarm_enable all 2 bytes long
//        I2C_WriteReg(i2c_controller, i2c_addr, command, TX_data, 2, timeout);
//        vTaskDelay(0);
//    }
//    return true;
//}

bool BQ769X2_PROTOCOL::sendDirectCommandR(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, CmdDrt command, uint16_t * readOut, TickType_t timeout){
    if(I2C_ReadReg(i2c_controller, i2c_addr, command, (uint8_t *)readOut, 2, timeout))
        vTaskDelay(2);
    else
        return false;

    return true;
}

bool BQ769X2_PROTOCOL::sendDirectCommandW(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, CmdDrt command, uint16_t data, TickType_t timeout) {
    uint8_t TX_data[2] = {0x00, 0x00};

    //little endian format
    TX_data[0] = data & 0xff;
    TX_data[1] = (data >> 8) & 0xff;

    //Control_status, alarm_status, alarm_enable all 2 bytes long
    if(I2C_WriteReg(i2c_controller, i2c_addr, command, TX_data, 2, timeout))
        vTaskDelay(2);
    else
        return false;

    return true;
}

bool BQ769X2_PROTOCOL::sendCommandSubcommand(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, Cmd command, TickType_t timeout)  //For Command only Subcommands
// See the TRM or the BQ76952 header file for a full list of Command-only subcommands
// All that this function do is formatting the transfer array then writing the array to hex 3E,
// the monitor will then operate based on the command.
{  //For DEEPSLEEP/SHUTDOWN subcommand you will need to call this function twice consecutively

    uint8_t TX_Reg[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = command & 0xff;
    TX_Reg[1] = (command >> 8) & 0xff;

    if(I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_Reg, 2, timeout))
        vTaskDelay(pdMS_TO_TICKS(2));
    else
        return false;
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

bool BQ769X2_PROTOCOL::sendSubcommandR(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, Cmd cmd, uint8_t readOut[32], TickType_t timeout) {
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    uint8_t TX_Reg[4]    = {0x00, 0x00, 0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = cmd & 0xff;
    TX_Reg[1] = (cmd >> 8) & 0xff;

    if(!I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_Reg, 2, timeout))
        return false;

    vTaskDelay(pdMS_TO_TICKS(2));

    if(!I2C_ReadReg(i2c_controller, i2c_addr, 0x40, readOut, 32, timeout))
        return false;

    return true;
}

bool BQ769X2_PROTOCOL::sendSubcommandW(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, Cmd cmd, uint8_t data, TickType_t timeout) {
    //security keys and Manu_data writes dont work with this function (reading these commands works)
    //max readback size is 32 bytes i.e. DASTATUS, CUV/COV snapshot
    uint8_t TX_Reg[4]    = {0x00, 0x00, 0x00, 0x00};
    uint8_t TX_Buffer[2] = {0x00, 0x00};

    //TX_Reg in little endian format
    TX_Reg[0] = cmd & 0xff;
    TX_Reg[1] = (cmd >> 8) & 0xff;

    //FET_Control, REG12_Control
    TX_Reg[2] = data & 0xff;
    if(!I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_Reg, 3, timeout))
        return false;
    vTaskDelay(pdMS_TO_TICKS(1));
    TX_Buffer[0] = Checksum(TX_Reg, 3);
    TX_Buffer[1] = 0x05;  //combined length of registers address and data
    if(!I2C_WriteReg(i2c_controller, i2c_addr, 0x60, TX_Buffer, 2, timeout))
        return false;
    vTaskDelay(pdMS_TO_TICKS(1));

    return true;
}

bool BQ769X2_PROTOCOL::sendSubcommandW2(System::I2C::I2C &i2c_controller,uint8_t i2c_addr, Cmd cmd, uint16_t data, TickType_t timeout) {
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
    if(!I2C_WriteReg(i2c_controller, i2c_addr, 0x3E, TX_Reg, 4, timeout))
        return false;
    vTaskDelay(pdMS_TO_TICKS(1));
    TX_Buffer[0] = Checksum(TX_Reg, 4);
    TX_Buffer[1] = 0x06;  //combined length of registers address and data
    if(!I2C_WriteReg(i2c_controller, i2c_addr, 0x60, TX_Buffer, 2, timeout))
        return false;
    vTaskDelay(pdMS_TO_TICKS(1));

    return true;
}

bool BQ769X2_PROTOCOL::setRegister(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, RegAddr reg_addr, uint32_t reg_data, uint8_t datalen, TickType_t timeout)
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

bool BQ769X2_PROTOCOL::I2C_ReadReg(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout)
{
    // tx read address then rx fetched
    return      i2c_controller.tx_blocking(i2c_addr, &reg_addr, 1, timeout)
            &&  i2c_controller.rx_blocking(i2c_addr, reg_data, count, timeout)
        ;
}

//bool BQ769X2_PROTOCOL::I2C_WriteReg(uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout)
//{
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
//             DL_I2C_CONTROLLER_STATUS_IDLE))
//        ;
//
//    DL_I2C_startControllerTransfer(I2C_0_INST, I2C_TARGET_ADDRESS,
//        DL_I2C_CONTROLLER_DIRECTION_TX, count + 1);
//
//    while (DL_I2C_getControllerStatus(I2C_0_INST) &
//           DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
//        ;
//    /* Wait for I2C to be Idle */
//    while (!(DL_I2C_getControllerStatus(I2C_0_INST) &
//             DL_I2C_CONTROLLER_STATUS_IDLE))
//        ;
//
//    //Avoid BQ769x2 to stretch the SCLK too long and generate a timeout interrupt at 400kHz because of low power mode
//    // if(DL_I2C_getRawInterruptStatus(I2C_0_INST,DL_I2C_INTERRUPT_CONTROLLER_CLOCK_TIMEOUT))
//    // {
//    //     DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//    //     DL_I2C_clearInterruptStatus(I2C_0_INST,DL_I2C_INTERRUPT_CONTROLLER_CLOCK_TIMEOUT);
//    //     I2C_WriteReg(reg_addr, reg_data, count);
//    // }
//    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
//
//    return true;
//}

bool BQ769X2_PROTOCOL::I2C_WriteReg(System::I2C::I2C &i2c_controller, uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count, TickType_t timeout)
{
    uint8_t I2Ctxbuff[8] = {0x00};

    I2Ctxbuff[0] = reg_addr;
    uint8_t i, j = 1;

    for (i = 0; i < count; i++) {
        I2Ctxbuff[j] = reg_data[i];
        j++;
    }

    return i2c_controller.tx_blocking(i2c_addr, I2Ctxbuff, count+1, timeout);
}

//************************************End of BQ769X2_PROTOCOL Measurement Commands******************************************
