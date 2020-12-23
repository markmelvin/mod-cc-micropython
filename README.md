# mod-cc-micropython
A micropython module implementing the moddevices control chain protocol (currently on an STM32 development board).

# Current status
The code currently handshakes and will show actuators in the Mod Duo(X) UI, but assignments have yet to be implemented. This is not difficult - I am just sharing my progress as I go along.

# Running on an STM32 NUCLEO-F446RE development board

Currently this project runs on an ST Microelectronics NUCLEO-F446RE development board with a [moddevices Arduino shield](https://www.moddevices.com/products/arduino-shield) attached.

## Dependencies

https://www.st.com/en/development-tools/stsw-link009.html

https://github.com/dhylands/rshell

https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads

https://github.com/stlink-org/stlink/releases/tag/v1.3.0

## Required micropython configuration

### Threading

The code in this repository uses threading which is not enabled by default. This can be enabled by defining `MICROPY_PY_THREAD` to `1` in the file `mpconfigport.h` for the port you are using (in our case, `ports/stms32/mpconfigport.h`), and rebuilding micropython.

### Timers

This code requires one high-speed timer and is configured to use TIM2 (which is normally used for a servo on the `stm32` port of micropython but we have no servos :) ).

### Serial configuration

In order to allow easy programming and debugging it is recommended that `UART2` be left as the REPL uart and `UART6` be selected as the communication link to the Mod Duo(X).  This means a small hardware modification is also required (see below).

## Required hardware configuration

### Power

The entire system can be powered from the Mod Duo(X). To configure this, simply change the `PWR` jumper to be on the `E5V` side.  It is safe to plug the USB cable into the development board for programming and debugging while the Mod Duo(X) is powering the system (and communicating), but there is one small caveat. **You must power the board from the ethernet jack (from the Mod Duo(X)) before plugging in the USB cable to the development board. If you plug the USB cable in first, the system will not power up correctly and just won't work.**

### Serial port

Since we are using `UART6` for communications to the Mod Duo(X), we need to make a small change to the Arduino shield and connect a jumper wire to make things work. This is because:

* The Arduino shield is configured to use `UART2` (which is shared with the USB connection to the PC)
* One pin from `UART6` is nto actually on the Arduino headers but is available on another header on the development board

You don't *have* to do this, but if you use (`UART6` for the debug/programming connection) you need to remove the shield and change the powering scheme every time you make a change - which is a total pain in the ass.  So, to make it work (and it is totally worth it for ease of debugging/programming), you need to do the following:

* Do **not** install header pins in the `TX` abd `RX` Arduino header positions on the Arduino shield. This avoids connecting the serial pins interfacing with the Mod Duo(X) with `UART2` on the development board (which we'll be using for debugging and flashing instead).
* Install a jumper between pin `-9` and pin `RX` on the Arduino shield
* Install a jumper from pin `4` to `CN10` on the development board to pin `TX` on the Arduino shield

This effectively jumpers the `UART6` pins over to the `RX ` and `TX` signals going to the Mod Duo(X).
