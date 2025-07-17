## TODO
1. Setup Thermister Demo > DONE
1. I2C wrapper : missing TRX > LOW PRIORITY (LATER)
1. UART wrapper : missing RX
	- RX blocking function
	- function to check if RX fifo is empty
1. UART CLI, consider using _FreeRTOS + CLI_
1. SPI wrapper : verify funcitonality
1. BQ protocol & troubleshooting
	- make the I2C code not hang with BQ clock streches. timeout of some sort
	- investigate password configuration, may get away with without
1. BQ wrapper
1. BQ class
1. BQ flush out core feature of the chip
	- BQ read thermisters test
	- BQ s/w controlled ldo test
1. prototype mcu & BQ chip
	- iterate design
	- test on E25
1. determin approach with SOC algo. custom or 3rd party?

## changes for next itteration of VT board
1. <s>correct can transceiver footprint</s>
1. <s>add TP to VCORE of MSP</s>
1. add silk screen lables on all jumpers, connections, buttons
1. <s>voltage output of can transceiver subsystem ranges from 4.5V to 5.5V as input voltage varies.</s> _(put a bigger buff cap)_.
1. <s>thermistors were wrong size; footprint is correct, part bought was wrong.</s>
1. <s>1206 is a good size, keep it</s>
1. <s>BLS invoke line from the XDS110 is not needed. figure out what its for. Y is it there if we dont need it.</s> _(bootstrap loader)_
1. consider increasing fill zone clearance around VT traces
1. <s>figure out and buy the debugger ribon cable connector thing, or get a clamp on for the pin headers</s>_(too expensive. do later or make edge connector)_
1. <s>bigger led's. its hard to solder :(</s>_(no choice)_
1. <s>jumper pad series resistors for cell simulation</s>_(no need, continue using TH)_
1. <s>exposed ground pad near edge for alligator clip</s>
1. <s>TP on both comm lines between BQ and MSP</s>
1. include uart comm settings on silkscreen
1. include version number on silkscreen
1. include git repo on silkscreen
1. include contributor names on silkscreen
1. <s>~5.5k 3v3 pull up on the i2c lines</s>
1. uart <-> usb method
1. series sense resistor. size for car?

## Would be nice to have done
1. FreeRTOS + tcp using the W5500
1. fatFS for SDCard datalogging
1. document elcon charger CAN interface
1. boot manager from UART
1. remove the rando from the VT github
1. figure out how to use the ROV debugger thing. :( allegedly pnp (is not).