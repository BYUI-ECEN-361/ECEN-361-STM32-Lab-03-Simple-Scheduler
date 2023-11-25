# ECEN-361-STM32-Lab-01-multiboard
## BYU-I  Department of Electrical and Computer Engineering
## Overview / Purpose
This sample code has been written to show how the functions added by the Multi-function board can be used for extended I/O with the STM32-Nucleo Board to be used in ECEN-361.

<img title="Multifunction Board" alt="Alt text" src="/Documentation/multifunction_shield-l1600.jpg">

Schematic for this board can be found [HERE](/Documentation/multifunction_sheild_schematics.pdf).

## Hardware Modification **BEFORE** Using
This multifunction shield has a conflict with the USART used by the STM32 boards for programming.  Pin D3 (Arduino) used for the Beeper on this board is pin PB5 on the STM32 and dedicated for communication to/from the IDE debugger as **SW1**.  As is, this conflict causes the buzzer to constantly be on, with data to/from the debugger port.  Rather than change the standard debugger used in the STM32Cube IDE, it's easier to re-connect the hardware used for the buzzer.  For now, simply disabling the buzzer can be done by cutting the conflicting output that goes to the buzzer. 

The location of the trace to be cut (XACTO knife) can be seen in the picture here:

<img title="Multifunction Board Trace Cut" alt="Alt text" src="/Documentation/20230813_trace-cut.jpg">

This trace leads from D3 to the base of the PNP transistor and simply disconnects the buzzer from running all the time with the debug data output from the ST-Link.

### Primary I/O Functions used in Demo
* LEDs (4 simple)
* Switches (3)
* Potentiometer


## Switches
 All of the switches are pulled-high with resistors on the board, and are active-low.  The three switches are set to do the following:
* **Switch-1:**
This switch toggles the direction of the counter.  The direction is an enumerated-type.  It changes based on the falling-edged.
* **Switch-2:**
This switch starts/stops the binary counter of the LEDs.
* **Switch-3:**
This switch presets the digit-counter to 5555.



## Seven-Segment LEDs
The four 7-Segment LEDs are driven thru a pair of serial->parallel shift registers.  Each register drives a couple of the LED's and the processor only has to dedicate a single data pin and clock pin to set it up.  The only obscure part of this configuration is that only a single 7-Segment LED gets turned on at a time.  By using a timer, the processor cycles thru the segments, and the persistence of the LEDs makes them appear to be on all the time.  See the code, Timer15 is the count-down timer and its ISR goes thru the 4 LEDs.

The Seven-Segment digits are programmed to start running count-up.  When they reach 9999 they roll-over to 0000.  The count-up/count-down behavior swaps, based on the SW-1.   Each press of SW-1 changes the direction of the count.  The counter is simply part of the main() infinite loop.

The speed of the count is determined by the voltage from the potentiometer.  This speed changes the wait time of the loop.  

## Potentiometer
The built-in 10K Poteniometer is wired as a voltage divider between 0V and 5V with the wiper (adjustable voltage) going to the A0 pin of the shield (or PA0 of the STM32 pinout, also the Morpho CN7-28.)  The ADC is set up to do a continuous read, in a blocking fashion, so the value isn't updated until the ADC process is completed each time.  The speed isn't an issue.

## UART for Serial Monitor
One of the timer is dedicated to put out a diagnostic message once a second.  This message goes out thru a 115,200 baud serial port connected thru the USB port to the PC.  This can be seen with a PuTTY or equivalent terminal emulator.

A timer initiates the diagnostic output time between messages.

## Programming thru CubeMX
Configuration of the GPIOs, Interrupts, and Timers is all done thru the built-in GUI .ioc file.  This generates all structure around user-code blocks.  The GUI can also be opened standalone with the STM32-CubeX program.

See also the include file with the exact #defines [here: "MultiFunctionShield.h"](/Core/Inc/MultiFunctionShield.h). 

Customizations for this multifunction board are:




<!-- >: /* is this a comment */ -->
