# mod-cc-micropython
A micropython module implementing the moddevices control chain protocol (currently on an STM32 development board), as well as a simple program that provides a footswitch implementation with four foot swtiches and corresponding indicator lights.

# Current status
The code is currently fully-functional. It handshakes and will show up in the Mod Duo(X) UI, and you can freely assign any toggle or option signal to any of the four switch inputs. Indication of the state of the four assignments is provided via four LED output signals.

Currently tap tempo is not supported but will be implemented soon.

# Running on an STM32 NUCLEO-F446RE development board

Currently this project runs on an ST Microelectronics NUCLEO-F446RE development board with a [moddevices Arduino shield](https://www.moddevices.com/products/arduino-shield) attached.

## Dependencies

If you want to use the exact same development board, you can use the provided firmware images (compiled with the latest `master` branch of micropython as of December 25th, 2020) for the micropython runtime. If you want to use a different board, you can compile micropython yourself (see [required-micropython-configuration]).

If you are using an ST-Micro development board with a connected ST-LINK you will need the following drivers/utilities (assuming Windows development, using WSL for Linux when needed) to flash the firmware :

### ST_LINK USB Driver
https://www.st.com/en/development-tools/stsw-link009.html

### Open source firmware flashing utility

NOTE: We're using an older version since the latest fails due to a bug

https://github.com/stlink-org/stlink/releases/tag/v1.3.0

## Required micropython configuration

If you plan on building micropython yourself from scratch, you will need [micropython](https://github.com/micropython/micropython) as well as the [ARM toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads).
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
* One pin from `UART6` is not actually on the Arduino headers but is available on another header on the development board

You don't *have* to do this, but if you instead use `UART6` for the debug/programming connection you need to remove the shield and change the powering scheme every time you make a change - which is a total pain in the ass.  So, to make it work (and it is totally worth it for ease of debugging/programming), you need to do the following:

* Do **not** install header pins in the `TX` abd `RX` Arduino header positions on the Arduino shield. This avoids connecting the serial pins interfacing with the Mod Duo(X) with `UART2` on the development board (which we'll be using for debugging and flashing instead).
* Install a jumper between pin `-9` and pin `RX` on the Arduino shield
* Install a jumper from pin `4` of `CN10` on the development board to pin `TX` on the Arduino shield

This effectively jumpers the `UART6` pins over to the `RX ` and `TX` signals going to the Mod Duo(X).

## Flashing the firmware

There are two steps to flashing the firmware:

### Flashing the micropython runtime

If using an ST Microelectronics NUCLEO-F446RE development board, you can use the provided images for the micropython runtime. If you have compiled micropython from scratch, follow the micropython directions for your board to flash the micropython runtime onto your device.

There is [a script in the root folder](./flash_micropython.bat) that uses a locally-installed `st-flash.exe` utility to flash the micropython firmware onto the device (assumes the ST-LINK driver is installed and the board is plugged in).

### Flashing the user program (written in Python)

Your user-program is written in pure micropython and is flashed separately from the micropython runtime. This is done with a simple serial connection to the micropython REPL using a utility called [rshell](https://github.com/dhylands/rshell). This must be installed and there are a couple of scripts provided to flash the user program onto the device. [flash_pyfootswitch.bat](./flash_pyfootswitch.bat) will flash all of the firmware (control chain protocol module and other helper modules) to the device as well as the main user program (`main.py`). This only needs to be done once (or whenever a helper module changes). The other script - [flash_main_py.bat](./flash_main_py.bat) - flashes the main user program to the device. Typically this all you need to flash on a regular basis during development.

Once everything has been flashed, you can plug your "footswitch" into your Mod Duo(X) and you should see it detected in the Mod web GUI!

## Customizing the footswitch

It is recommended you edit `main.py` to customize the name and URL of the device as well as any DIO pin names such that they correspond to your specific board/device.