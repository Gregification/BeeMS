## TODO
1. I2C wrapper : missing TRX _(LOW PRIORITY)_
1. UART CLI, consider using _FreeRTOS + CLI_
1. SPI wrapper : verify funcitonality
1. BQ protocol & troubleshooting
	- make the I2C code not hang with BQ clock streches. timeout of some sort
	- investigate password configuration, may get away with without
1. BQ wrapper
1. BQ class
1. BQ flush out core feature of the chip
	- <s>BQ read thermisters test (done)</s>
	- BQ s/w controlled ldo test
	- use CRC on the I2C if viable
1. prototype mcu & BQ chipFCLK
	- iterate design
	- test on E25
1. determin approach with SOC algo. custom or 3rd party?
1. use the MSPM0G RTC for time stamping. until this is done just use hte FRTOS system tick.

## changes for next itteration of VT board
1. use diode footprint from D6 on all LEDs
1. add gnd tap poitns for all isolated sections
1. double check trace width calculations to hte hsunt resister : see saturn 
1. skilscreen note for what pin is for LED
1. use different led colors for stuff
1. investigate why the uart-usb bridge circuit crapped out
1. add external high frequency oscillator for CAN

## Would be nice to have done
1. FreeRTOS + tcp using the W5500
1. fatFS for SDCard datalogging
1. document elcon charger CAN interface
1. boot manager from UART
1. remove the rando from the VT github
1. figure out how to use the ROV debugger thing. :( allegedly pnp (is not).