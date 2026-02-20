# CEB (CAN-bus Ethernet Bridge)
Translates CAN/CAN-FD packets to a custom IP packet. The custom IP packet can then be translated back to CAN/CAN-FD at a remote CAN bus. No limit on number of endpoints.
> it is possible to bridge between CAN-buses of different speeds.

## to use
> one CEB is to be attatched to each CAN-bus network you intend to join. The following steps are to be preformed for each device.
1. plug in the [GLV connector](#glv-connector).
1. plug in the [ethernet connector](#ethernet-connector).
1. wait for blue light to start blinking. should be nearly instant when power is applied.
    - indicates the RTOS is running
1. when a message is passed along the bridge, the green light will blink.
    - it may take up to ~20 seconds for the ethernet side to initialize on power-up.

> different network settings can be applied by editing software and reflashing device. see [Application Notes section](#application-notes) for more.


## Application Notes
- no onboard CAN-bus terminating resistor.
- Does not support multidrop or PoE ethernet.
- there is notable packet latency, >~1mS, caused by the ethernet interface. Not all CAN-bus devices can work with such latency.
- UDP broadcast packets (e.g: 192.168.1.__255__ | SN: 255.255.255.0) are used. Some networking equiptment block broadcasts by default.
    - do not have more than 1 CEB on a CAN-bus network. They WILL loop back on eachother and DDOS the CAN bus.
- baud rates up to 8MBs , baud is hardcoded in software.
    - see <a href="https://github.com/Gregification/BeeMS/blob/main/CAN_ETH_TRX_MSPM0G/src/Core/system.cpp">`system.cpp`</a> for CAN controller baud setup.
- static IP based on 32b MCU factory serial number. Dumped on UART during boot.
    - see <a href="https://github.com/Gregification/BeeMS/blob/main/CAN_ETH_TRX_MSPM0G/src/Core/CETRX.hpp">`CETRX.hpp`</a> for networking configuration (static IP, broadcast IP, MAC, etc). 
    - `192.168.1.x`
        - x: lower B of MCU serial number.
    - MAC: `BE:EE:EE:x:y:y`
        - x: upper B of MCU serial number.
        - y: lower B of MCU serial number.
- UART is intended for TTY output at 8N1 115200.

## Software Modification
- All code involved is embeded code running on the <a href="https://www.ti.com/product/MSPM0G3507-Q1">MSPM0G3507</a> TI MCU.
- C/C++ , c17, c++17
- software at <a href="https://github.com/Gregification/BeeMS/tree/main/CAN_ETH_TRX_MSPM0G">CAN_ETH_TRX_MSPM0G</a>. 
    - IDE: Code Composer Studio (CCS) v12 . 
        - TI Clang compiler v4.0.4 installed though CCS
    - SDK: Driver-Lib SDK v2.05.00.05 . a newer version can be used but you'll have to update all the references
    - SWD & UART pin out on board for programming and debugging.
- MCU has a unique 32b hardware level serial number from factory.
- runs FreeRTOS
    - main loop task is in <a href="https://github.com/Gregification/BeeMS/blob/main/CAN_ETH_TRX_MSPM0G/src/Tasks/task_CAN_Eth_TRX.hpp">`task_CAN_Eth_TRX.hpp`</a><a href="https://github.com/Gregification/BeeMS/blob/main/CAN_ETH_TRX_MSPM0G/src/Tasks/task_CAN_Eth_TRX.cpp">`cpp`</a>.
- CAN bus speed settings are in <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/system.cpp">system.cpp</a> . If the option your looking for is not there here are your options.
    1.  (suggested) use <a href="www.ti.com/tool/SYSCONFIG">SysConfig</a>
        1. open <a href="../../80mhzjunk.syscfg">this SysConfig project</a> 
        1. set the desired CAN baud 
        1. find the output c file and copy the settings
    1. do it by hand. See the <a href="https://www.ti.com/lit/pdf/slau846">technical manual</a>. 

#### Files

<a href="../../CAN_ETH_TRX_MSPM0G/src/">../../CAN_ETH_TRX_MSPM0G/src</a><br>
├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/">Core</a><br>
│   ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/CETRX.cpp">CETRX.cpp</a> initing board specific peripherials<br>
│   ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/CETRX.hpp">CETRX.hpp</a> board specific MCU peripherials<br> 
│   ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/common.c">common.c</a><br>
│   ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/common.h">common.h</a> common macros and globals<br>
│   ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/main.cpp">main.cpp</a><br>
│   ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/std%20alternatives/">std alternatives</a> std like functions without the memory overhead<br>
│   │   ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/std%20alternatives/string.cpp">string.cpp</a><br>
│   │   └── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/std%20alternatives/string.hpp">string.hpp</a><br>
│   ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/system.cpp">system.cpp</a> initing most of the MCU peripherials<br>
│   ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/system.hpp">system.hpp</a> MCU hardware configurations <br>
│   └── <a href="../../CAN_ETH_TRX_MSPM0G/src/Core/system_decls.cpp"> system_decls.cpp</a><br>
├── <a href="../../CAN_ETH_TRX_MSPM0G/src/FreeRTOSConfig.h">FreeRTOSConfig.h</a><br>
├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Middleware/">Middleware</a><br>
│   └── <a href="../../CAN_ETH_TRX_MSPM0G/src/Middleware/W5500/">W5500</a> ethernet controller<br>
│   &nbsp;&nbsp;&nbsp; ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Middleware/W5500/socket.c">socket.c</a><br>
│   &nbsp;&nbsp;&nbsp; ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Middleware/W5500/socket.h">socket.h</a><br>
│   &nbsp;&nbsp;&nbsp; ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Middleware/W5500/w5500.c">w5500.c</a><br>
│   &nbsp;&nbsp;&nbsp; ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Middleware/W5500/w5500.h">w5500.h</a><br>
│   &nbsp;&nbsp;&nbsp; ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Middleware/W5500/wizchip_conf.c">wizchip_conf.c</a><br>
│   &nbsp;&nbsp;&nbsp; └── <a href="../../CAN_ETH_TRX_MSPM0G/src/Middleware/W5500/wizchip_conf.h">wizchip_conf.h</a><br>
└── <a href="../../CAN_ETH_TRX_MSPM0G/src/Tasks/">Tasks</a><br>
&nbsp;&nbsp;&nbsp; ├── <a href="../../CAN_ETH_TRX_MSPM0G/src/Tasks/task_CAN_Eth_TRX.cpp">task_CAN_Eth_TRX.cpp</a> main bridge logic<br>
&nbsp;&nbsp;&nbsp; └── <a href="../../CAN_ETH_TRX_MSPM0G/src/Tasks/task_CAN_Eth_TRX.hpp">task_CAN_Eth_TRX.hpp</a><br>

### How To Program
Any SWD capiable device will do. a binary file can be generated by CCS.
The following instructions are just one of many ways to do it.

Use a TI XDS110 launchpad (<a href="https://www.ti.com/tool/LP-MSPM0G3507">example</a>), and disconnect the target MCU side of the launchpad by removing all the onboard jumpers. Use a wire jumper to connect the pins on the XDS110 side of the launchpad to the matching pins on the CEB (lables on silkscreen). Plug in the launchpad USB to the computer running CCS(Code Composer Studio). In CCS click the 'flash' or 'debug' option to upload your program to the device. Afterwards you can disconnect the pins. __DO NOT change wiring while both launch pad and target board are powered. unpower at least one of them.__ Its possible to ignore this step but you risk destroying either device, to save seconds of time.

## Connectors
### GLV connector
- molex microfit 6 pin (2x3) header (part#: 43045-0600). <a href="https://www.mouser.com/ProductDetail/Molex/43045-0600?qs=mrPiglD9aYJ8CzJ%2FE2PNbw%3D%3D&srsltid=AfmBOop7-nikRF2atEfFslZtuPWfqFOihNLqcdVlYseRCGCFOiFBx6Nk">mouser</a>.

![meow](../zzLocalLibraries-Symbols/Microfit%20header%20pin%20numbers%202x3.png)
1. Interlock side A. Internally shorted to pin 4.
    - not required
2. GND
3. CAN low
4. Interlock side B. Internally shorted to pin 1.
    - not required
5. LV+ (up to 35V)
6. CAN high

### Ethernet connector
- molex microfit 6 pin (2x3) header (part#: RJMG1BD3B8K1ANR, mouser#: 523-RJMG1BD3B8K1ANR ). <a href="https://www.mouser.com/ProductDetail/Amphenol-Commercial-Products/RJMG1BD3B8K1ANR?qs=8nwx9VzGl4z0N6DynlxHHg%3D%3D">mouser.</a>
- <a href="https://cdn.amphenol-cs.com/media/wysiwyg/files/drawing/rjmg1bd3b8k1anr.pdf">mechanical drawing</a>

![meow](../zzLocalLibraries-Symbols/rj45%20header%20TH%20pin%20numbers.png)
- Audo-MDIX is not supported by this device. Just use any standard ethernet wiring and let the other device figure it out.
- 10/100Mbps capiable
