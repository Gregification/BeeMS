# Voltage Tap integrated Orion Temperature Sensor
A custom implimentation of the <a href="https://www.orionbms.com/products/thermistor-expansion-module/">Orion TEM</a>, integrated into to the E27 voltage tap board. Replces the need for a ~30 wire thermistor harness for each module with a 6 wire daisy chainable connector.

The mini-TEM (this project) is a voltage tap board with a MCU on it. Instead of passing the thermistors to a external harness the thermistor information is measured onboard then transmitted digitally over CAN-bus (CAN-FD capiable). The <a href="https://www.orionbms.com/downloads/misc/thermistor_module_canbus.pdf">TEM CAN packet format</a> is simple and well documented by the OEM. This board - mini-TEM - will abide by this standard and therfore is compatiable with the Orion BMS.

## TODO
    1. add interlock relay. (replay symbol is already on schematic, just needs to be wired.) will need to find a new isoaltor, or a varient of it to get the needed channels
    1. finish layout. Thermistor locations must line up with whats in solid works. Ask grant about the dimentions.

## notes
- SOFTWARE IS A WORK IN PROGRESS
- no onboard CAN-bus terminating resistor.
- software at <a href="https://github.com/Gregification/BeeMS/tree/main/hardware/mini%20Orion%20TEM">mini Orion TEM</a>. Code Composer project. SWD & UART pinout on board for programming and debugging. TI Clang compiler and Driver-Lib SDK is needed.
- baud rates up to 8MBs , baud is hardcoded in software.
    - see <a href="link">`TOOD update: system.cpp`</a> for CAN controller baud setup.
- MCU has a unique 32b hardware level serial number from factory.
- runs FreeRTOS
    - main loop task is in <a href="link">`TOOD : task_CAN_Eth_TRX.hpp`</a>.

## connectors
### GLV Connector 
- molex microfit 6 pin (2x3) header (part#: 43045-0600). <a href="https://www.mouser.com/ProductDetail/Molex/43045-0600?qs=mrPiglD9aYJ8CzJ%2FE2PNbw%3D%3D&srsltid=AfmBOop7-nikRF2atEfFslZtuPWfqFOihNLqcdVlYseRCGCFOiFBx6Nk">mouser</a>.

![microfit 2x6 header](../zzLocalLibraries-Symbols/Microfit%20header%20pin%20numbers%202x3.png)
1. Interlock side A. shorted to pin 4.
2. GND
3. CAN low
4. Interlock side B. shorted to pin 1.
5. LV+ (up to 35V)
6. CAN high

### Voltage Tap Connector
- 13761132

