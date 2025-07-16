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
1. correct can transceiver footprint
1. add TP to VCORE of MSP
1. add silk screen lables on all jumpers
1. voltage output of can transceiver subsystem ranges from 4.5V to 5.5V as input voltage varies. find why, it should be a constant 5V
	- also swap out the funky coil inductor with some potted one, very noisy
1. thermistors were wrong size; footprint is correct, part bought was wrong.
1. 1206 is a good size, keep it
1. BLS invoke line from the XDS110 is not needed. figure out what its for. Y is it there if we dont need it.
1. consider increasing fill zone clearance around VT traces
1. figure out and buy the debugger ribon cable connector thing, or get a clamp on for the pin headers
1. bigger led's. its hard to solder :(
1. jumper pad series resistors for cell simulation
1. exposed ground pad near edge for alligator clip
1. TP on both comm lines between BQ and MSP
1. include uart comm settings on silkscreen
1. include version number on silkscreen
1. include git repo on silkscreen
1. include contributor names on silkscreen
1. ~5.5k 3v3 pull up on the i2c lines

## Would be nice to have done
1. FreeRTOS + tcp using the W5500
1. fatFS for SDCard datalogging
1. document elcon charger CAN interface
1. boot manager from UART
1. remove the rando from the VT github
1. figure out how to use the ROV debugger thing. :( allegedly pnp (is not).