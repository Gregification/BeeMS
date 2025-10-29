/*
 * BQ796x2_PROTOCOL_Test_V.cpp
 *
 *  Created on: Jul 13, 2025
 *      Author: FSAE
 */

#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <ti/driverlib/driverlib.h>
#include <Tasks/examples/example_BQ769x2_PROTOCOL_V.hpp>
#include "Middleware/BQ769x2/BQ76952.hpp"
#include "Core/system.hpp"

void printBattStatus(uint16_t);
void printSafteyStatusA(uint8_t);
void printSafteyStatusB(uint8_t);
void printSafteyStatusC(uint8_t);
void printPFStatusA(uint8_t); // permanent failure alert
void printPFStatusB(uint8_t); // permanent failure alert
void printPFStatusC(uint8_t); // permanent failure alert
void printPFStatusD(uint8_t); // permanent failure alert
void printVCellMode(uint16_t);

#ifdef PROJECT_ENABLE_I2C1
void Task::BQ769x2_PROTOCOL_Test_V_Task(void*) {
    System::uart_ui.nputs(ARRANDN("BQ769x2_PROTOCOL_Test_V_Task Start" NEWLINE));

    char str[MAX_STR_LEN_COMMON];

    // -----------------------------------------------------------------------------

    BQ76952 bq;
    bq.i2c_controller   = &System::i2c1;
    bq.i2c_addr         = 0x8;

    bq.i2c_controller->setSCLTarget(100e3);

    vTaskDelay(pdMS_TO_TICKS(500));

    while(0) {
        static uint8_t arr[] = {1,2,3,4,5,6,7,8,9};
        static const uint8_t arr_size = sizeof(arr);
        static char output_buffer[64];
        static volatile bool res;
//        res = bq.i2c_controller->rx_blocking(0x08, arr, sizeof(arr), 0);
        bq.i2c_controller->rx(0x08, arr, sizeof(arr));
//        vTaskDelay(pdMS_TO_TICKS(100));
//        res = bq.i2c_controller->tx_blocking(0x08, arr, sizeof(arr), 0 );
//        vTaskDelay(pdMS_TO_TICKS(100));

        if(res) System::uart_ui.nputs(ARRANDN(CLIRESET CLIGOOD));
        else    System::uart_ui.nputs(ARRANDN(CLIRESET CLIBAD));

        // --- INLINED PRINTING LOGIC START ---

        // Use an offset to track the current position in the output_buffer.
        int offset = 0;

        // Start the string with a descriptive prefix.
        offset += snprintf(output_buffer + offset, sizeof(output_buffer) - offset, "RX_DATA: ");

        // 2. LOOP TO FORMAT RECEIVED BYTES
        // This replaces the original fragmented printing loop.
        for(int k = 0; k < arr_size; k++){
            // Check boundary before writing
            if (offset >= sizeof(output_buffer) - 4) { // Need at least 4 chars (space, 2 hex digits, null terminator)
                break;
            }

            // Format the current byte (arr[k]) as two-digit hexadecimal (%02X) followed by a space.
            offset += snprintf(output_buffer + offset, sizeof(output_buffer) - offset, "%02X ", arr[k]);
        }

        // 3. FINISH AND OUTPUT THE STRING

        // Replace the trailing space with a newline character for clean console output.
        // We only do this if we actually appended data (offset > "RX_DATA: ".length).
        if (offset > 9) {
            output_buffer[offset - 1] = '\0';
            output_buffer[offset] = '\0';
        } else {
            // Safety measure: ensure it's null-terminated even if no data was written.
            output_buffer[offset] = '\0';
        }

        // Output the complete, formatted string in one go.
        // Assuming System::uart_ui.nputs can handle the final null-terminated string.
        System::uart_ui.nputs(ARRANDN(output_buffer));

        System::uart_ui.nputs(ARRANDN(NEWLINE));
    }

    // -----------------------------------------------------------------------------

//    { // reset device
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "resetting device ... "));
//        if(bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::BQ769x2_RESET))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success"));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail"));
//        System::uart_ui.nputs(ARRANDN(NEWLINE));
//        vTaskDelay(pdMS_TO_TICKS(100));
//    }
//
//    { // dump control status
//        uint16_t v = 0xbeef;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "dumping control status ... "));
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::ControlStatus, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                System::uart_ui.nputs(ARRANDN(NEWLINE "print control status: "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//
//                System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "[15:3] reserved"));
//
//                // --- Bit 2: DEEPSLEEP ---
//                System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Device DEEPSLEEP mode? "));
//                if((v & BV(2)))     System::uart_ui.nputs(ARRANDN(CLIYES "in DEEPSLEEP"));
//                else                System::uart_ui.nputs(ARRANDN(CLINO "not in DEEPSLEEP"));
//
//                // --- Bit 1: LD_TIMEOUT (Load Detect Timeout) ---
//                System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Load Detect (LD) timeout? "));
//                if((v & BV(1)))     System::uart_ui.nputs(ARRANDN(CLIYES "timed out/deactivated"));
//                else                System::uart_ui.nputs(ARRANDN(CLINO "not timed out/active"));
//
//                // --- Bit 0: LD_ON (Load Detect Pullup Status) ---
//                System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "LD pullup active? "));
//                if((v & BV(0)))     System::uart_ui.nputs(ARRANDN(CLIYES "active"));
//                else                System::uart_ui.nputs(ARRANDN(CLINO "not active"));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        System::uart_ui.nputs(ARRANDN(NEWLINE));
//    }
//
//    { // enable 16 cell mode & dump cell config
//        uint16_t v;
//
//        v = 0xFF; // enable all cells
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "VCellMode ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::VCellMode, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        v = 0xbeef;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::VCellMode, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                printVCellMode(v);
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//    }
//
//    { // configure pins as thermistor connections
//        /*ALERT, CFETOFF, DFETOFF, TS1, TS2, TS3, HDQ, DCHG, DDSG*/
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Configuring to thermistor inputs ... "));
//
//        uint8_t v;
//
//        v = 0b11;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "\tALERT  ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::ALERTPinConfig, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::ALERTPinConfig, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        v = 0b11;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "\tCFETOFF  ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::CFETOFFPinConfig, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::CFETOFFPinConfig, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        v = 0b11;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "\tDFETOFF  ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::DFETOFFPinConfig, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::DFETOFFPinConfig, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        v = 0b11;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "\tTS1  ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::TS1Config, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::TS1Config, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        v = 0b11;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "\tTS2  ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::TS2Config, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::TS2Config, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        v = 0b11;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "\tTS3  ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::TS3Config, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::TS3Config, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        v = 0b11;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "\tHDQ  ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::HDQPinConfig, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::HDQPinConfig, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        v = 0b11;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "\tDCHG  ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::DCHGPinConfig, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::DCHGPinConfig, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        v = 0b11;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "\tDDSG  ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::DDSGPinConfig, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::DDSGPinConfig, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        System::uart_ui.nputs(ARRANDN(NEWLINE));
//    }
//
//    { // disable & dump Permanent Failure register masks
//        uint8_t v;
//        v = 0x00;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "disabling PFA ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::EnabledPFA, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::EnabledPFA, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        v = 0x00;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "disabling PFB ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::EnabledPFB, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::EnabledPFB, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        v = 0x00;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "disabling PFC ... "));
//        if(bq.I2C_WriteReg((uint8_t)BQ769X2_PROTOCOL::RegAddr::EnabledPFC, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::EnabledPFC, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        v = 0x00;
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "disabling PFD ... "));
//        if(bq.I2C_WriteReg(BQ769X2_PROTOCOL::RegAddr::EnabledPFD, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "tx success . "));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "tx fail . "));
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::RegAddr::EnabledPFD, &v, sizeof(v))){
//                System::uart_ui.nputs(ARRANDN(CLIGOOD "rx success "));
//                snprintf(ARRANDN(str), "0x%02x", v);
//                System::uart_ui.nputs(ARRANDN(str));
//        }
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD "rx fail"));
//
//        System::uart_ui.nputs(ARRANDN(CLIRESET NEWLINE));
//    }
//
//    { // dump status regs
//        uint8_t v;
//
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::SafetyStatusA, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD));
//        printSafteyStatusA(v);
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::SafetyStatusB, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD));
//        printSafteyStatusB(v);
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::SafetyStatusC, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD));
//        printSafteyStatusC(v);
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::PFStatusA, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD));
//        printPFStatusA(v); // permanent failure alert
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::PFStatusB, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD));
//        printPFStatusB(v); // permanent failure alert
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::PFStatusC, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD));
//        printPFStatusC(v); // permanent failure alert
//        v = 0xbe;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::PFStatusD, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD));
//        printPFStatusD(v); // permanent failure alert
//
//        System::uart_ui.nputs(ARRANDN(NEWLINE));
//    }
//
//    { // raw alarm status
//        uint16_t v;
//
//        v = 0xbeef;
//        if(bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::AlarmRawStatus, &v, sizeof(v)))
//                System::uart_ui.nputs(ARRANDN(CLIGOOD));
//        else    System::uart_ui.nputs(ARRANDN(CLIBAD));
//
//        System::uart_ui.nputs(ARRANDN("AlarmRawStatus: "));
//        snprintf(ARRANDN(str), "0x%04x", v);
//        System::uart_ui.nputs(ARRANDN(str));
//
//        // --- Bit 15: SSBC (Safety Status B/C Set) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Safety Status B/C set (SSBC)? "));
//        if((v & BV(15)))    System::uart_ui.nputs(ARRANDN(CLIYES "SET"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "not set"));
//
//        // --- Bit 14: SSA (Safety Status A Set) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Safety Status A set (SSA)? "));
//        if((v & BV(14)))    System::uart_ui.nputs(ARRANDN(CLIYES "SET"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "not set"));
//
//        // --- Bit 13: PF (Permanent Fail Triggered) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Permanent Fail fault (PF)? "));
//        if((v & BV(13)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "none"));
//
//        // --- Bit 12: MSK_SFALERT (Masked Safety Alert) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Masked Safety Alert (MSK_SFALERT)? "));
//        if((v & BV(12)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "none"));
//
//        // --- Bit 11: MSK_PFALERT (Masked Permanent Fail Alert) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Masked PF Alert (MSK_PFALERT)? "));
//        if((v & BV(11)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "none"));
//
//        // --- Bit 10: INITSTART (Initialization started) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Initialization started (INITSTART)? "));
//        if((v & BV(10)))    System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "no"));
//
//        // --- Bit 9: INITCOMP (Initialization completed) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Initialization completed (INITCOMP)? "));
//        if((v & BV(9)))     System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "no"));
//
//        // --- Bit 8: RSVD (Reserved/Undefined) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Reserved bit (8): "));
//        if((v & BV(8)))     System::uart_ui.nputs(ARRANDN(CLIYES "1"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "0"));
//
//        // --- Bit 7: FULLSCAN (Full Voltage Scan Complete) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Full Voltage Scan Complete (FULLSCAN)? "));
//        if((v & BV(7)))     System::uart_ui.nputs(ARRANDN(CLIYES "complete"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "in progress"));
//
//        // --- Bit 6: XCHG (Charge FET Off) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "CHG FET Off (XCHG)? "));
//        if((v & BV(6)))     System::uart_ui.nputs(ARRANDN(CLIYES "off"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "on"));
//
//        // --- Bit 5: XDSG (Discharge FET Off) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "DSG FET Off (XDSG)? "));
//        if((v & BV(5)))     System::uart_ui.nputs(ARRANDN(CLIYES "off"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "on"));
//
//        // --- Bit 4: SHUTV (Stack voltage too low) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Stack voltage below shutdown threshold (SHUTV)? "));
//        if((v & BV(4)))     System::uart_ui.nputs(ARRANDN(CLIYES "LOW"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "OK"));
//
//        // --- Bit 3: FUSE (FUSE Pin Driven) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "FUSE Pin Driven (FUSE)? "));
//        if((v & BV(3)))     System::uart_ui.nputs(ARRANDN(CLIYES "driven"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "not driven"));
//
//        // --- Bit 2: CB (Cell Balancing Active) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell Balancing (CB) active? "));
//        if((v & BV(2)))     System::uart_ui.nputs(ARRANDN(CLIYES "active"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "inactive"));
//
//        // --- Bit 1: ADSCAN (Voltage ADC Scan Complete) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Voltage ADC Scan Complete (ADSCAN)? "));
//        if((v & BV(1)))     System::uart_ui.nputs(ARRANDN(CLIYES "complete"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "in progress"));
//
//        // --- Bit 0: WAKE (Wake from SLEEP) ---
//        System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Device WOKE from SLEEP (WAKE)? "));
//        if((v & BV(0)))     System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
//        else                System::uart_ui.nputs(ARRANDN(CLINO "no"));
//
//        System::uart_ui.nputs(ARRANDN(NEWLINE));
//
//    }
//
//    {
//        uint16_t v = 0xBEEF;
//
//        // dump battery status
//        bool success = bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::BatteryStatus, &v, sizeof(v));
//        System::uart_ui.nputs(ARRANDN(success ? CLIGOOD : CLIBAD));
//        printBattStatus(v);
//        System::uart_ui.nputs(ARRANDN(CLIRESET));
//
//        vTaskDelay(pdMS_TO_TICKS(1e3));
//        System::uart_ui.nputs(ARRANDN(NEWLINE));
//
//    }
//
//    if(0)
//    {
//        bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::SLEEP_DISABLE);
//    }
//
//
//    { // SEAL -> UNSEAL
//        //bqTM.13.8.2/197
//
//        /* bqTM.8.1/71
//         *  """
//         *      each transition requires that a unique set of keys be sent to the device
//         *      through the sub-command address (0x3E and 0x3F). The keys must be sent
//         *      consecutively to 0x3E and 0x3F, with no other data written between the
//         *      keys. Do not set the two keys to identical values
//         *  """
//         */
//
//        uint32_t usk = 0x3672'0414; // un-seal key
////        uint32_t usk = 0x3672'0414; // un-seal key : factory default
//
//        uint8_t sk1[] = {0x3E, (uint8_t)((usk & 0x0000'00FF) >> 00), (uint8_t)((usk & 0x0000'FF00) >> 8)};
//        bq.i2c_controller->tx_blocking(bq.i2c_addr, sk1, sizeof(sk1), pdMS_TO_TICKS(10));
//
//        uint8_t sk2[] = {0x3E, (uint8_t)((usk & 0x00FF'0000) >> 16), (uint8_t)((usk & 0xFF00'0000) >> 24)};
//        bq.i2c_controller->tx_blocking(bq.i2c_addr, sk2, sizeof(sk2), pdMS_TO_TICKS(10));
//    }
//
//
//    if(0)
//    { // TODO: DOES NOT WORK!!!!!!!!!    UNSEAL -> FULL-ACCESS
//        uint32_t fak = 0xFFFF'FFFF; // full access key
////        uint32_t fak = 0xFFFF'FFFF'; // full access key : factory default
//
//        uint8_t sk1[] = {0x3E, (uint8_t)((fak & 0x0000'00FF) >> 00), (uint8_t)((fak & 0x0000'FF00) >> 8)};
//        bq.i2c_controller->tx_blocking(bq.i2c_addr, sk1, sizeof(sk1), pdMS_TO_TICKS(10));
//
//        uint8_t sk2[] = {0x3E, (uint8_t)((fak & 0x00FF'0000) >> 16), (uint8_t)((fak & 0xFF00'0000) >> 24)};
//        bq.i2c_controller->tx_blocking(bq.i2c_addr, sk2, sizeof(sk2), pdMS_TO_TICKS(10));
//    }
//
//    System::uart_ui.nputs(ARRANDN(NEWLINE));
//
//    {
//        uint16_t v = 0xBEEF;
//
//        // dump battery status
//        bool success = bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::BatteryStatus, &v, sizeof(v));
//        System::uart_ui.nputs(ARRANDN(success ? CLIGOOD : CLIBAD));
//        printBattStatus(v);
//        System::uart_ui.nputs(ARRANDN(CLIRESET));
//
//        vTaskDelay(pdMS_TO_TICKS(10e3));
//        System::uart_ui.nputs(ARRANDN(NEWLINE NEWLINE));
//
//    }


    // -----------------------------------------------------------------------------

    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::BQ769x2_RESET);
    vTaskDelay(pdMS_TO_TICKS(61));

    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::SET_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(9));


    // After entering CONFIG_UPDATE mode, RAM registers can be programmed. When programming RAM, checksum and length must also be
    // programmed for the change to take effect. All of the RAM registers are described in detail in the BQ769x2 TRM.
    // An easier way to find the descriptions is in the BQStudio Data Memory screen. When you move the mouse over the register name,
    // a full description of the register and the bits will pop up on the screen.

    // 'Power Config' - 0x9234 = 0x2D80
    // Setting the DSLP_LDO bit allows the LDOs to remain active when the device goes into Deep Sleep mode
    // Set wake speed bits to 00 for best performance
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::PowerConfig, 0x2D80, 2);

    // 'REG0 Config' - set REG0_EN bit to enable pre-regulator
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::REG0Config, 0x01, 1);

    // 'REG12 Config' - Enable REG1 with 3.3V output (0x0D for 3.3V, 0x0F for 5V)
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::REG12Config, 0x0D, 1);

    // Set DFETOFF pin to control BOTH CHG and DSG FET - 0x92FB = 0x42 (set to 0x00 to disable)
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::DFETOFFPinConfig, 0x42, 1);

    // Set up ALERT Pin - 0x92FC = 0x2A
    // This configures the ALERT pin to drive high (REG1 voltage) when enabled.
    // The ALERT pin can be used as an interrupt to the MCU when a protection has triggered or new measurements are available
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::ALERTPinConfig, 0x2A, 1);

    // Set TS1 to measure Cell Temperature - 0x92FD = 0x07
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::TS1Config, 0x07, 1);

    // Set TS3 to measure FET Temperature - 0x92FF = 0x0F
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::TS3Config, 0x0F, 1);

    // Set HDQ to measure Cell Temperature - 0x9300 = 0x07
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::HDQPinConfig, 0x00, 1);  // No thermistor installed on EVM HDQ pin, so set to 0x00

    // 'VCell Mode' - Enable 16 cells - 0x9304 = 0x0000; Writing 0x0000 sets the default of 16 cells
    // Only for openwire detection and  protection
    uint16_t u16TempValue = 0;
    for (uint8_t u8Count = 0; u8Count < (16 - 1); u8Count++) {
        u16TempValue += (0x1 << u8Count);
    }
    u16TempValue += 0x8000;
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::VCellMode, u16TempValue, 2);

    // Enable protections in 'Enabled Protections A' 0x9261 = 0xBC
    // Enables SCD (short-circuit), OCD1 (over-current in discharge), OCC (over-current in charge),
    // COV (over-voltage), CUV (under-voltage)
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledProtectionsA, 0xBC, 1);

    // Enable all protections in 'Enabled Protections B' 0x9262 = 0xF7
    // Enables OTF (over-temperature FET), OTINT (internal over-temperature), OTD (over-temperature in discharge),
    // OTC (over-temperature in charge), UTINT (internal under-temperature), UTD (under-temperature in discharge), UTC (under-temperature in charge)
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::EnabledProtectionsB, 0xF7, 1);

    // 'Default Alarm Mask' - 0x..82 Enables the FullScan and ADScan bits, default value = 0xF800
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::DefaultAlarmMask, 0xF882, 2);

    // Set up Cell Balancing Configuration - 0x9335 = 0x03   -  Automated balancing while in Relax or Charge modes
    // Also see "Cell Balancing with BQ769x2 Battery Monitors" document on ti.com
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::BalancingConfiguration, 0x03, 1);

    //Set the minimum cell balance voltage in charge - 0x933B = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinCellVCharge, 3.3 - 100, 2);
//        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);
    //Set the minimum cell balance voltage in rest - 0x933F = pBattParamsCfg->u16MinFullChgVoltThd_mV-100 mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::CellBalanceMinCellVRelax, 3.3 - 100, 2);
//        pBattParamsCfg->u16MinFullChgVoltThd_mV - 100, 2);

    // Set up CUV (under-voltage) Threshold - 0x9275 = 0x31 (2479 mV)
    // CUV Threshold is this value multiplied by 50.6mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::CUVThreshold, 0x31, 1);
//    BQ769x2_SetRegister(
//        CUVThreshold, pBattParamsCfg->u16MinBattVoltThd_mV / 51, 1);

    // Set up COV (over-voltage) Threshold - 0x9278 = 0x55 (4301 mV)
    // COV Threshold is this value multiplied by 50.6mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::COVThreshold, 0x55, 1);
//    BQ769x2_SetRegister(
//        COVThreshold, pBattParamsCfg->u16MaxBattVoltThd_mV / 51, 1);

    // Set up OCC (over-current in charge) Threshold - 0x9280 = 0x05 (10 mV = 10A across 1mOhm sense resistor) Units in 2mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::OCCThreshold, 0x05, 1);
//    BQ769x2_SetRegister(
//        OCCThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up OCD1 (over-current in discharge) Threshold - 0x9282 = 0x0A (20 mV = 20A across 1mOhm sense resistor) units of 2mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::OCD1Threshold, 0x0A, 1);
//    BQ769x2_SetRegister(
//        OCD1Threshold, pBattParamsCfg->i16MinDhgCurtThd_mA / 2000, 1);

    // Set up SCD (short discharge current) Threshold - 0x9286 = 0x05 (100 mV = 100A across 1mOhm sense resistor)  0x05=100mV
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::SCDThreshold, 0x05, 1);
//    BQ769x2_SetRegister(
//        SCDThreshold, pBattParamsCfg->i16MaxChgCurtThd_mA / 2000, 1);

    // Set up SCD Delay - 0x9287 = 0x03 (30 us) Enabled with a delay of (value - 1) * 15 us; min value of 1
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::SCDDelay, 0x03, 1);

    // Set up SCDL Latch Limit to 1 to set SCD recovery only with load removal 0x9295 = 0x01
    // If this is not set, then SCD will recover based on time (SCD Recovery Time parameter).
    bq.setRegister(BQ769X2_PROTOCOL::RegAddr::SCDLLatchLimit, 0x01, 1);


    vTaskDelay(pdMS_TO_TICKS(9));
    // Exit CONFIGUPDATE mode  - Subcommand 0x0092
    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::EXIT_CFGUPDATE);
    vTaskDelay(pdMS_TO_TICKS(9));
    //Control All FETs on
    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::FET_ENABLE);
    vTaskDelay(pdMS_TO_TICKS(9));
    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::ALL_FETS_ON);
    vTaskDelay(pdMS_TO_TICKS(9));
    bq.sendCommandSubcommand(BQ769X2_PROTOCOL::Cmd::SLEEP_DISABLE);
    vTaskDelay(pdMS_TO_TICKS(9));

    // -----------------------------------------------------------------------------

    const BQ769X2_PROTOCOL::CmdDrt cmds[] = {
             BQ769X2_PROTOCOL::CmdDrt::Cell1Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell2Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell3Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell4Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell5Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell6Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell7Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell8Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell9Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell10Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell12Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell13Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell14Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell15Voltage,
             BQ769X2_PROTOCOL::CmdDrt::Cell16Voltage
        };

//    while(1)
//    {
//        uint16_t v = 0xBEEF;
//
//        // dump battery status
//        bool success = bq.I2C_ReadReg(BQ769X2_PROTOCOL::CmdDrt::BatteryStatus, &v, sizeof(v));
//        System::uart_ui.nputs(ARRANDN(success ? CLIGOOD : CLIBAD));
//        printBattStatus(v);
//        System::uart_ui.nputs(ARRANDN(CLIRESET));
//
//        vTaskDelay(pdMS_TO_TICKS(5e3));
//        System::uart_ui.nputs(ARRANDN(NEWLINE NEWLINE NEWLINE));
//    }


    while(true){
        for(uint8_t i = 0; i < sizeof(cmds); i++){
            uint16_t v = 0xBEEF;
            bool success = bq.I2C_ReadReg(cmds[i], &v, sizeof(v));

            snprintf(ARRANDN(str), "%6d,", v);

            System::uart_ui.nputs(ARRANDN(success ? CLIGOOD : CLIBAD));
            System::uart_ui.nputs(ARRANDN(str));
            System::uart_ui.nputs(ARRANDN(CLIRESET));

            vTaskDelay(pdMS_TO_TICKS(1e3));
        }
        System::uart_ui.nputs(ARRANDN(NEWLINE));
    }

    System::uart_ui.nputs(ARRANDN("BQ769x2_PROTOCOL_Test_V_Task End" NEWLINE));
    vTaskDelete(NULL);
}
#else
void Task::BQ769x2_PROTOCOL_Test_V_Task(void*){
    vTaskDelete(NULL);
};
#endif

void printBattStatus(uint16_t v) {
    System::uart_ui.nputs(ARRANDN(NEWLINE "printBattStatus: "));
    {
        char str[10];
        snprintf(ARRANDN(str), "0x%04x", v);
        System::uart_ui.nputs(ARRANDN(str));
    }

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "CFG_UPDATE mode ? "));
    if((v & BV(0)))    System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "no"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"PCHG_UPDATE mode ? "));
    if((v & BV(1)))    System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "no"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"SLEEP_en ? "));
    if((v & BV(2)))    System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "no"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"POR ? "));
    System::uart_ui.nputs(ARRANDN((v & BV(3)) ? "1" : "0"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"WD : "));
    if((v & BV(4)))    System::uart_ui.nputs(ARRANDN("previous reset caused by WD"));
    else               System::uart_ui.nputs(ARRANDN("previous reset normal"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"COW_CHK (Cell Open Wire Check active)? "));
    if((v & BV(5)))    System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "no"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"OTPW (OTP write pending)? "));
    if((v & BV(6)))    System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "no"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"OTPB (OTP write blocked)? "));
    if((v & BV(7)))    System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "no"));

    // SEC bits (9:8)
    uint8_t sec = (v >> 8) & 0x03;
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Security mode: "));
    switch (sec) {
        case 0:
            System::uart_ui.nputs(ARRANDN("Not initialized"));
            break;
        case 1:
            System::uart_ui.nputs(ARRANDN("FULLACCESS"));
            break;
        case 2:
            System::uart_ui.nputs(ARRANDN("UNSEALED"));
            break;
        case 3:
            System::uart_ui.nputs(ARRANDN("SEALED"));
            break;
    }

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"FUSE pin asserted? "));
    if((v & BV(10)))    System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "no"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Safety fault (SS)? "));
    if((v & BV(11)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "none"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Permanent Fail (PF)? "));
    if((v & BV(12)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "none"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Shutdown pending (SDM)? "));
    if((v & BV(13)))    System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "no"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Reserved bit (14): "));
    System::uart_ui.nputs(ARRANDN((v & BV(14)) ? "1" : "0"));

    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Device in SLEEP mode? "));
    if((v & BV(15)))    System::uart_ui.nputs(ARRANDN(CLIYES "yes"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "no"));

    System::uart_ui.nputs(ARRANDN(NEWLINE));
}

void printSafteyStatusA(uint8_t v) {
    System::uart_ui.nputs(ARRANDN(NEWLINE "printSafteyStatusA: "));
    {
        char str[10];
        snprintf(ARRANDN(str), "0x%02x", v);
        System::uart_ui.nputs(ARRANDN(str));
    }

    // --- Bit 7: SCD (Short Circuit in Discharge) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Short Circuit in Discharge (SCD) fault? "));
    if((v & BV(7)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 6: OCD2 (Overcurrent in Discharge 2nd Tier) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Overcurrent in Discharge 2nd Tier (OCD2) fault? "));
    if((v & BV(6)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 5: OCD1 (Overcurrent in Discharge 1st Tier) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Overcurrent in Discharge 1st Tier (OCD1) fault? "));
    if((v & BV(5)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 4: OCC (Overcurrent in Charge) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Overcurrent in Charge (OCC) fault? "));
    if((v & BV(4)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 3: COV (Cell Overvoltage Protection) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Cell Overvoltage (COV) fault? "));
    if((v & BV(3)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 2: CUV (Cell Undervoltage Protection) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Cell Undervoltage (CUV) fault? "));
    if((v & BV(2)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 1: Reserved bit (1) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Reserved bit (1): "));
    System::uart_ui.nputs(ARRANDN((v & BV(1)) ? "1" : "0"));

    // --- Bit 0: Reserved bit (0) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Reserved bit (0): "));
    System::uart_ui.nputs(ARRANDN((v & BV(0)) ? "1" : "0"));

    System::uart_ui.nputs(ARRANDN(NEWLINE));
}

void printSafteyStatusB(uint8_t v) {
    System::uart_ui.nputs(ARRANDN(NEWLINE "printSafteyStatusB: "));
    {
        char str[10];
        snprintf(ARRANDN(str), "0x%02x", v);
        System::uart_ui.nputs(ARRANDN(str));
    }

    // --- Bit 7: OTF (FET Overtemperature) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"FET Overtemperature (OTF) fault? "));
    if((v & BV(7)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 6: OTINT (Internal Overtemperature) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET"Internal Overtemperature (OTINT) fault? "));
    if((v & BV(6)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 5: OTD (Overtemperature in Discharge) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Overtemperature in Discharge (OTD) fault? "));
    if((v & BV(5)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 4: OTC (Overtemperature in Charge) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Overtemperature in Charge (OTC) fault? "));
    if((v & BV(4)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 3: Reserved bit (3) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Reserved bit (3): "));
    System::uart_ui.nputs(ARRANDN((v & BV(3)) ? "1" : "0"));

    // --- Bit 2: UTINT (Internal Undertemperature) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Internal Undertemperature (UTINT) fault? "));
    if((v & BV(2)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 1: UTD (Undertemperature in Discharge) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Undertemperature in Discharge (UTD) fault? "));
    if((v & BV(1)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 0: UTC (Undertemperature in Charge) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Undertemperature in Charge (UTC) fault? "));
    if((v & BV(0)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    System::uart_ui.nputs(ARRANDN(NEWLINE));
}

void printSafteyStatusC(uint8_t v) {
    System::uart_ui.nputs(ARRANDN(NEWLINE "printSafteyStatusC: "));
    {
        char str[10];
        snprintf(ARRANDN(str), "0x%02x", v);
        System::uart_ui.nputs(ARRANDN(str));
    }

    // --- Bit 7: OCD3 (Overcurrent in Discharge 3rd Tier) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Overcurrent in Discharge 3rd Tier (OCD3) fault? "));
    if((v & BV(7)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 6: SCDL (Short Circuit in Discharge Latch) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Short Circuit in Discharge (SCDL) latch fault? "));
    if((v & BV(6)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 5: OCDL (Overcurrent in Discharge Latch) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Overcurrent in Discharge (OCDL) latch fault? "));
    if((v & BV(5)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 4: COVL (Cell Overvoltage Latch) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell Overvoltage (COVL) latch fault? "));
    if((v & BV(4)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 3: Reserved bit (3) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Reserved bit (3): "));
    System::uart_ui.nputs(ARRANDN((v & BV(3)) ? "1" : "0"));

    // --- Bit 2: PTO (Precharge Timeout) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Precharge Timeout (PTO) fault? "));
    if((v & BV(2)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 1: HWDF (Host Watchdog Fault) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Host Watchdog (HWDF) fault? "));
    if((v & BV(1)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 0: Reserved bit (0) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Reserved bit (0): "));
    System::uart_ui.nputs(ARRANDN((v & BV(0)) ? "1" : "0"));

    System::uart_ui.nputs(ARRANDN(NEWLINE));
}

// permanent failure alert
void printPFStatusA(uint8_t v) {
    System::uart_ui.nputs(ARRANDN(NEWLINE "printPFStatusA: "));
    {
        char str[10];
        snprintf(ARRANDN(str), "0x%02x", v);
        System::uart_ui.nputs(ARRANDN(str));
    }

    // --- Bit 7: CUDEP (Copper Deposition Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Copper Deposition (CUDEP) permanent fail? "));
    if((v & BV(7)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 6: SOTF (Safety Overtemperature FET Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Safety Overtemperature FET (SOTF) permanent fail? "));
    if((v & BV(6)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 5: Reserved bit (5) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Reserved bit (5): "));
    System::uart_ui.nputs(ARRANDN((v & BV(5)) ? "1" : "0"));

    // --- Bit 4: SOT (Safety Overtemperature Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Safety Overtemperature (SOT) permanent fail? "));
    if((v & BV(4)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 3: SOCD (Safety Overcurrent in Discharge Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Safety Overcurrent in Discharge (SOCD) permanent fail? "));
    if((v & BV(3)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 2: SOCC (Safety Overcurrent in Charge Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Safety Overcurrent in Charge (SOCC) permanent fail? "));
    if((v & BV(2)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 1: SOV (Safety Cell Overvoltage Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Safety Cell Overvoltage (SOV) permanent fail? "));
    if((v & BV(1)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 0: SUV (Safety Cell Undervoltage Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Safety Cell Undervoltage (SUV) permanent fail? "));
    if((v & BV(0)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    System::uart_ui.nputs(ARRANDN(NEWLINE));
}

// permanent failure alert
void printPFStatusB(uint8_t v) {
    System::uart_ui.nputs(ARRANDN(NEWLINE "printPFStatusB: "));
    {
        char str[10];
        snprintf(ARRANDN(str), "0x%02x", v);
        System::uart_ui.nputs(ARRANDN(str));
    }

    // --- Bit 7: SCDL (Short Circuit in Discharge Latch Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Short Circuit Discharge Latch (SCDL) permanent fail? "));
    if((v & BV(7)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 6: Reserved bit (6) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Reserved bit (6): "));
    System::uart_ui.nputs(ARRANDN((v & BV(6)) ? "1" : "0"));

    // --- Bit 5: Reserved bit (5) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Reserved bit (5): "));
    System::uart_ui.nputs(ARRANDN((v & BV(5)) ? "1" : "0"));

    // --- Bit 4: VIMA (Voltage Imbalance Active Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Voltage Imbalance Active (VIMA) permanent fail? "));
    if((v & BV(4)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 3: VIMR (Voltage Imbalance at Rest Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Voltage Imbalance at Rest (VIMR) permanent fail? "));
    if((v & BV(3)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 2: 2LVL (Second Level Protector Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Second Level Protector (2LVL) permanent fail? "));
    if((v & BV(2)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 1: DFETF (Discharge FET Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Discharge FET (DFETF) permanent fail? "));
    if((v & BV(1)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 0: CFETF (Charge FET Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Charge FET (CFETF) permanent fail? "));
    if((v & BV(0)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    System::uart_ui.nputs(ARRANDN(NEWLINE));
}

// permanent failure alert
void printPFStatusC(uint8_t v) {
    System::uart_ui.nputs(ARRANDN(NEWLINE "printPFStatusC: "));
    {
        char str[10];
        snprintf(ARRANDN(str), "0x%02x", v);
        System::uart_ui.nputs(ARRANDN(str));
    }

    // --- Bit 7: CMDF (Commanded Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Commanded Permanent Fail (CMDF)? "));
    if((v & BV(7)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 6: HWMX (Hardware Mux Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Hardware Mux (HWMX) permanent fail? "));
    if((v & BV(6)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 5: VSSF (Internal VSS Measurement Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Internal VSS Measurement (VSSF) permanent fail? "));
    if((v & BV(5)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 4: VREF (Internal Voltage Reference Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Internal Voltage Reference (VREF) permanent fail? "));
    if((v & BV(4)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 3: LFOF (Internal LFO Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Internal LFO (LFOF) permanent fail? "));
    if((v & BV(3)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 2: IRMF (Instruction ROM Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Instruction ROM (IRMF) permanent fail? "));
    if((v & BV(2)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 1: DRMF (Data ROM Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Data ROM (DRMF) permanent fail? "));
    if((v & BV(1)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    // --- Bit 0: OTPF (OTP Memory Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "OTP Memory (OTPF) permanent fail? "));
    if((v & BV(0)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    System::uart_ui.nputs(ARRANDN(NEWLINE));
}

// permanent failure alert
void printPFStatusD(uint8_t v) {
    System::uart_ui.nputs(ARRANDN(NEWLINE "printPFStatusD: "));
    {
        char str[10];
        snprintf(ARRANDN(str), "0x%02x", v);
        System::uart_ui.nputs(ARRANDN(str));
    }

    // bits 7:1 reserved

    // --- Bit 0: TOSF (Top of Stack vs Cell Sum Permanent Fail) ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Top of Stack vs Cell Sum (TOSF) permanent fail? "));
    if((v & BV(0)))    System::uart_ui.nputs(ARRANDN(CLIYES "triggered"));
    else               System::uart_ui.nputs(ARRANDN(CLINO "none"));

    System::uart_ui.nputs(ARRANDN(NEWLINE));
}

void printVCellMode(uint16_t v) {
    System::uart_ui.nputs(ARRANDN(NEWLINE "printVCellMode: "));
    {
        char str[10];
        snprintf(ARRANDN(str), "0x%02x", v);
        System::uart_ui.nputs(ARRANDN(str));
    }

    // --- Bit 15: Cell 16 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 16 Mode: "));
    if((v & BV(15)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 14: Cell 15 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 15 Mode: "));
    if((v & BV(14)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 13: Cell 14 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 14 Mode: "));
    if((v & BV(13)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 12: Cell 13 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 13 Mode: "));
    if((v & BV(12)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 11: Cell 12 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 12 Mode: "));
    if((v & BV(11)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 10: Cell 11 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 11 Mode: "));
    if((v & BV(10)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 9: Cell 10 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 10 Mode: "));
    if((v & BV(9)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 8: Cell 9 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 9 Mode: "));
    if((v & BV(8)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 7: Cell 8 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 8 Mode: "));
    if((v & BV(7)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 6: Cell 7 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 7 Mode: "));
    if((v & BV(6)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 5: Cell 6 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 6 Mode: "));
    if((v & BV(5)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 4: Cell 5 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 5 Mode: "));
    if((v & BV(4)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 3: Cell 4 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 4 Mode: "));
    if((v & BV(3)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 2: Cell 3 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 3 Mode: "));
    if((v & BV(2)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 1: Cell 2 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 2 Mode: "));
    if((v & BV(1)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    // --- Bit 0: Cell 1 Mode ---
    System::uart_ui.nputs(ARRANDN(NEWLINE CLIRESET "Cell 1 Mode: "));
    if((v & BV(0)))    System::uart_ui.nputs(ARRANDN(CLIYES "enabled"));
    else                System::uart_ui.nputs(ARRANDN(CLINO "disabled"));

    System::uart_ui.nputs(ARRANDN(NEWLINE));
}
