# VITOS 
> Voltage Integrated Termperature Orion Sensor System

> yeah i fluffed the name. VTIOTS(Voltage Tap integrated Orion Temperature Sensor) just dosent sound nice

A custom implimentation of the <a href="https://www.orionbms.com/products/thermistor-expansion-module/">Orion TEM</a>, integrated into to the E27 voltage tap board. Replces the need for a ~30 wire thermistor harness for each module with a 6 wire daisy chainable connector.

The mini-TEM (this project) is a voltage tap board with a MCU on it. Instead of passing the thermistors to a external harness the thermistor information is measured onboard then transmitted digitally over CAN-bus (CAN-FD capiable). This board abides by the <a href="https://www.orionbms.com/downloads/misc/thermistor_module_canbus.pdf">TEM CAN packet format</a> and therefore is compatiable with the Orion BMS as a thrid party TEM device.

___
- 2 layers of working electrical isolation. no board cuts, under conformal coating. 6.7mm of isolation of all sections from `HV`; 4mm of isolation between `LV` and `HRLV`.
    - galvanic isolation between `GLV`(Global LV) and `HV`(Tractive System / Pack). This board has an additional layer of galvanic isolation between the thermistors(`LV`) and `GLV` connector. Incase of damage to thermistor packaging. Effectively there are 2 'LV' regions and 1 'HV' region, all regions are >600Vdc isolated from oneanother.
        - `HRLV`/`GLV`: High risk LV. The GLV connector. The term`HRLV`is not FSAE required or defined.
        - `LV` : LV used in thermistor biasing. Is not FASE required or defined.
        - `HV`/`Tractive System`/`Battery Pack` : the big dog. the one that will explode you. This shares no signals with the other isolation regions. 
    - 707Vdc working isolation between `LV` and PWM output. (PWM output is same potential as `HRLV` when onboard jumpers assembled).
        - 707Vdc : BJT opto emulator. <a href="https://www.ti.com/product/ISOM8110-Q1">ISOM8110-Q1</a>
    - 637Vdc working isolation betweeen `LV` and `HRLV`.
        - 637Vdc : CAN transcevier. <a href="https://www.ti.com/product/ISO1044">ISO1044</a>
        - 1kVdc : DC/DC converter. <a href="https://www.ti.com/product/UCC33420-Q1">UCC33420-Q1</a>
- Cooling control PWM : A onboard LV(user supplied) PWM output - (2, 35)V <35A max - is provided to signal cooling action (e.g: fan). Cooling need is determined only by cell termerature. A higher duty cycle (active high) indicates greater cooling requirement. Acheived by isolated high side PMOS switching.
- Powered by LV connector, not by battery stack. ~%38 energey effecient (D:).
    - PDN
        - [LV] -> [RPP diode (920mVf)] -> [100mA CAN trx],[%88 buck] -> [%40 DC/DC isolator] -> [40mA MCU],[20mA CAN iso]
- Orion BMS TEM compatiable CAN bus packets
- Multiple CAN baud rates without reprogramming, 5000Kbs, 1Mbs, 500KbsA/2MbsD , 1MbsA/4MbsD. (hardware switch configured, automatic baud detiction possible but its a pain to set up - i did not).
- Modbus compatiable (up to 64B MosbusTCP packets supported) (custom ModbusTCP <-> CAN-FD translation required, see <a href="https://github.com/Gregification/BeeMS/blob/main/hardware/CAN_Eth_Bridge/info.md">CEB(CAN-bus Ethernet Bridge)</a>).
- %90 soldering iron-able. excluding: DC/DC converter and PWM PMOS, no lead SMD - hotplate/heat gun required.
- wire must be ran from the LV connector to the PWM power inputs. Wire as in literally lay 2 wires on top of the board and solder them in place (2.54mm pitch TH connectors, see [Assembly Process](#assembly-process)).

## TODO
    1. (software) modbus, rapid scada

## notes
- SOFTWARE IS A WORK IN PROGRESS
- no onboard CAN-bus terminating resistor.
- software at <a href="https://github.com/Gregification/BeeMS/tree/main/hardware/mini%20Orion%20TEM">mini Orion TEM</a>. Code Composer project. SWD & UART pinout on board for programming and debugging. TI Clang compiler and Driver-Lib SDK is needed.
- baud rates up to 8MBs , baud is hardcoded in software.
    - see <a href="link">`TOOD update: system.cpp`</a> for CAN controller baud setup.
- MCU has a unique 32b hardware level serial number from factory.
- runs FreeRTOS
    - main loop task is in <a href="link">`TOOD : task_CAN_Eth_TRX.hpp`</a>.

## Theory Of Operation
cell terminals are directly passed to the main voltage tap conenector, all cell inputs are fuzed for ~1.5A; nothing else on this board involves it. Remaining complexity is for termperature sensing.

FSAE rules require temperature monitoring  of >%20 of negative cell terminals. Each board is responsible for 28 cells total; 24 thermistors are on the board; 23 measure unique cell terminals (%82 coverage), 1 measures arbartary user defined location. 

Thermistors grouped in batches of 8, each batch has a 8:1 analog mux, output is tied to <a href="https://www.ti.com/product/MSPM0G3507-Q1">MCU (MSPM0G35)</a> ADC (12b) input that has a intergrated 40k pull-up. Each MUX has 3 selection pins (DI), 1 analog output. Software loop scans all thermistors, then broadcasts cell temperatures to CAN bus; the user defined temperature is broadcasted seperatly for other uses. 

## Assembly Process
> Hand Assembly. we have sweat shop at home. follow steps in order. Mid assembly verificaiton steps are included. 

Heatgun is prefered, hotplate alternative ok, careful of damaging previously assembled components.

1. Assemble: Heatgun
    1. U102 : GLV DC/DC converter
1. Assemble: Iron 
    1. C101 : IC required 
    1. C117 : bulk input
    1. C115 : bulk output
    1. D101 : GLV PWR RPP stage
    1. J101 : GLV connector
1. Test: GLV DC/DC converter : multi-meter
    1. Connect GLV connector
    1. verify (4.8, 5.2)V between `GLV-`(-) and `R101 pad 1`(+).
    1. Disconnect GLV
1. Assemble: Heatgun
    1. U103 : isolated DC/DC converter
1. Assemble: Iron 
    1. C103 : bulk output
    1. C111 : bulk output
    1. R101 : IC required
1. Test: isolated DC/DC converter : multi-meter
    1. Connect GLV connector
    1. verify (4.8, 5.2)V across `C111 pad 0`(-) and `C111 pad 1`(+).
    1. Disconnect GLV
1. Assemble: Heatgun
    1. Q101 : PWM high side PMOS
1. Test: mosfet solder job : multi-meter
    1. verify no contenuity between these nets
        1. `R113 pad 0`
        1. `R113 pad 1`
        1. `J106 pin 2 (fan+)`
        1. `J106 pin 1 (fan-)`
1. Assemble: Iron 
    1. go crazy. assemble all remaining componetns.
1. Assemble: Iron , adding fan PWM PWR
    1. Use wire rated for `HV` and fan current to connect these pins
        1. `J104 pin 1 (GLV+)` to `J2 pin 2 (PWR+)`
        1. `J104 pin 2 (GLV-)` to `J2 pin 1 (PWR-)`
        1. `J106 pin 1 (FAN-)` to ___user fan-___
        1. `J106 pin 2 (FAN+)` to ___user fan+___
1. Assemble: Sharpie
    1. scribble on the front of board a identifier(unique number, doodle, etc), date of assembly, and your name so we know who to blame when the pack cooks off. Leave room for other text.
1. Test: hardware functionality : software IDE
    1. go edit the code to test that the PWM works and all thermistors can be read.
        - software is a CCS project, SWD pins available on board. <a href="https://www.youtube.com/watch?v=RMlM1w4uO9M">TODO: CCS project here. also, as close as you get to mondern good sci-fy</a>.
    1. if testing fails go fix it.
1. Assemble: Sharpie
    1. scribble on the date of testing (when it passes).

## Connectors
### GLV Connector 
- molex microfit 6 pin (2x3) header (part#: 43045-0600). <a href="https://www.mouser.com/ProductDetail/Molex/43045-0600?qs=mrPiglD9aYJ8CzJ%2FE2PNbw%3D%3D&srsltid=AfmBOop7-nikRF2atEfFslZtuPWfqFOihNLqcdVlYseRCGCFOiFBx6Nk">mouser</a>.

![microfit 2x6 header](../zzLocalLibraries-Symbols/Microfit%20header%20pin%20numbers%202x3.png)
1. Interlock side A. shorted to pin 4.
2. GND
3. CAN low
4. Interlock side B. shorted to pin 1.
5. LV+ . (2, 35)V working, 36V MAX, RPP
6. CAN high

### Voltage Tap Connector
- 13761132
- see schematic for pinning.
