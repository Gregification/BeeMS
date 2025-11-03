/*
 * task_CC.cpp
 *
 * Single-purpose FreeRTOS task to read BQ76952 coulomb-counter data
 * and maintain a software-integrated mAh accumulator. Also provides
 * optional CAN telemetry packing.
 *
 * How to use:
 *   1) Ensure BQ76952 helpers exist (in your BQ76952 class):
 *        - bool read_DAStatus5(uint8_t out32[32]);
 *        - bool read_DAStatus6(uint8_t out32[32]); // optional
 *        - bool reset_passq();                    // optional
 *   2) When you create the task, pass a pointer to your BQ76952
 *      instance via pvParameters (see start_CC_task() below).
 *   3) Fill the TRM-specific constants (OFFSETS + LSBs) once.
 *
 * Notes:
 *   - We read CC2 (instantaneous current) and CC1 (device integrator / PassQ)
 *     from the DASTATUS5 block. Exact offsets + scaling come from the TRM.
 *   - Software mAh integration (Σ I·dt) is maintained as a cross-check.
 */

#include <stdint.h>
#include <string.h>
#include <cstdio>   // for std::snprintf

#include "FreeRTOS.h"
#include "task.h"

#include "Core/system.hpp"         // System::uart_ui, NEWLINE, etc.
#include "Middleware/BQ769x2/BQ76952.hpp"            // Your device wrapper (ensure include path)

// Optional: if you're using TI MCAN drivers for telemetry
#include <ti/driverlib/driverlib.h>

namespace Task {

// ======== Fill these from the BQ76952 TRM / your calibration ==========
// Scaling for CC2 Current(): microamps per LSB (after current-gain calib)
static constexpr int32_t CURRENT_LSB_uA_PER_LSB = 1000;   // TODO: set from TRM/cal

// Offsets inside the 32-byte DASTATUS5 block (big-endian fields)
static constexpr uint8_t CC2_CURRENT_OFF = 0x02;          // TODO: set from TRM (int16)
static constexpr uint8_t CC1_PASSQ_OFF   = 0x06;          // TODO: set from TRM (24/32b)

// CC1 (PassQ) scaling. Pick the one that matches your TRM; leave the other 0.
static constexpr double PASSQ_LSB_C_PER_LSB   = 0.0;      // e.g., 1e-6  (C/LSB)
static constexpr double PASSQ_LSB_mAh_PER_LSB = 1e-6;     // e.g., 1e-6  (mAh/LSB)

// CAN settings (optional). Set to 0 to disable TX.
static constexpr uint32_t CAN_ID_CC_TELEM = 0x100;        // standard 11-bit ID
static constexpr uint32_t CAN_TX_PERIOD_MS = 100;         // 100 ms

// ======================================================================

struct CC_Readings {
    int32_t current_mA = 0;   // from CC2 Current()
    double  passQ_mAh  = 0;   // from device integrator (CC1), if configured
    double  sw_mAh     = 0;   // software Σ I·dt
    uint32_t cc1_raw   = 0;   // raw CC1 for debugging
};

static inline double passq_raw_to_mAh(uint32_t raw)
{
    if (PASSQ_LSB_mAh_PER_LSB > 0.0) {
        return raw * PASSQ_LSB_mAh_PER_LSB;
    } else {
        // If TRM provides coulombs/LSB instead
        double c = raw * PASSQ_LSB_C_PER_LSB;  // C
        return c / 3.6;                        // 1 mAh = 3.6 C
    }
}

static bool read_cc_once(BQ76952& bq, CC_Readings& out,
                         TickType_t* last_tick /* may be null */)
{
    uint8_t buf[32] = {0};
    if (!bq.read_DAStatus5(buf)) return false;

    // --- CC2 Current(): int16, big-endian ---
    int16_t cc2_raw = (int16_t)((buf[CC2_CURRENT_OFF] << 8) | buf[CC2_CURRENT_OFF + 1]);
    out.current_mA = (int32_t)((int64_t)cc2_raw * CURRENT_LSB_uA_PER_LSB / 1000);

    // --- CC1 PassQ: width per TRM (example: 24-bit unsigned) ---
    uint32_t cc1_raw = ((uint32_t)buf[CC1_PASSQ_OFF] << 16) |
                       ((uint32_t)buf[CC1_PASSQ_OFF + 1] << 8) |
                       ((uint32_t)buf[CC1_PASSQ_OFF + 2] << 0);
    out.cc1_raw  = cc1_raw;
    out.passQ_mAh = passq_raw_to_mAh(cc1_raw);

    // --- Optional: software Σ I·dt ---
    if (last_tick) {
        TickType_t now = xTaskGetTickCount();
        if (*last_tick != 0) {
            double dt_h = (double)(now - *last_tick) / configTICK_RATE_HZ / 3600.0;
            out.sw_mAh += (double)out.current_mA * dt_h;   // mA * h -> mAh
        }
        *last_tick = now;
    }

    return true;
}

static void maybe_send_can(const CC_Readings& cc)
{
    if (CAN_ID_CC_TELEM == 0) return; // disabled

    DL_MCAN_TxBufElement tx{};
    tx.id  = CAN_ID_CC_TELEM;
    tx.rtr = 0; tx.xtd = 0; tx.esi = 0; tx.brs = 0; tx.fdf = 0; tx.efc = 0; tx.mm = 0;
    tx.dlc = 8; // 8 bytes payload

    // Pack: int32 current_mA + float mAh
    int32_t i_mA = cc.current_mA;
    float   mAh  = (float)cc.passQ_mAh; // or cc.sw_mAh
    memcpy(&tx.data[0], &i_mA, 4);
    memcpy(&tx.data[4], &mAh,  4);

    DL_MCAN_TxFIFOStatus tf{};
    DL_MCAN_getTxFIFOQueStatus(CANFD0, &tf);
    uint32_t bufferIndex = tf.putIdx;
    DL_MCAN_writeMsgRam(CANFD0, DL_MCAN_MEM_TYPE_FIFO, bufferIndex, &tx);
    DL_MCAN_TXBufAddReq(CANFD0, tf.getIdx);
}

void CC_task(void* pv)
{
    BQ76952* bq = static_cast<BQ76952*>(pv);
    configASSERT(bq != nullptr);

    System::uart_ui.nputs(ARRANDN("CC_task: start" NEWLINE));

    // Optional: start a fresh device-side integration window
    // bq->reset_passq();

    CC_Readings cc{};
    TickType_t last_tick = 0;

    const TickType_t poll_ticks = pdMS_TO_TICKS(10); // 10 ms poll for CC2 dynamics
    TickType_t last_can = xTaskGetTickCount();

    for (;;) {
        if (read_cc_once(*bq, cc, &last_tick)) {
            char line[160];
            std::snprintf(line, sizeof(line),
                     "I=%ld mA | PassQ(dev)=%.6f mAh | mAh(sw)=%.6f | CC1raw=%lu" NEWLINE,
                     (long)cc.current_mA, cc.passQ_mAh, cc.sw_mAh, (unsigned long)cc.cc1_raw);
            System::uart_ui.nputs(ARRANDN(line));
        }

        // Periodic CAN telemetry
        TickType_t now = xTaskGetTickCount();
        if ((now - last_can) >= pdMS_TO_TICKS(CAN_TX_PERIOD_MS)) {
            maybe_send_can(cc);
            last_can = now;
        }

        vTaskDelay(poll_ticks);
    }
}

// Convenience creator; call this from your system bring-up code.
BaseType_t start_CC_task(BQ76952* bq,
                         const char* name = "CC",
                         uint32_t stackWords = 768,
                         UBaseType_t prio = tskIDLE_PRIORITY + 2)
{
    return xTaskCreate(CC_task, name, stackWords, (void*)bq, prio, nullptr);
}

} // namespace Task
