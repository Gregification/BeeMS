#ifndef SRC_CORE_WDT_HPP_
#define SRC_CORE_WDT_HPP_

#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>

namespace WDT {

    enum TaskBit : uint32_t {
        BMS         = (1 << 0),
        ETH_MODBUS  = (1 << 1),
        SAMPLER     = (1 << 2),
        // add bits here as you add tasks

        ALL_TASKS   = BMS | ETH_MODBUS | SAMPLER,
    };

    extern volatile uint32_t _checkinMask;

    void init();        // call at end of System::init()
    void kick();        // called ONLY by watchdog_task
    void checkin(TaskBit bit);  // called by each task once per loop
    bool allCheckedIn();
    void resetCheckins();

}


#endif /* SRC_CORE_WDT_HPP_ */
