set COM_PORT=COM6
set MPY_CROSS=../micropython/mpy-cross/mpy-cross
set MPY_CROSS_ARGS=-march=armv7m

del .\py_cc\*.mpy
del .\utils\*.mpy

wsl %MPY_CROSS% %MPY_CROSS_ARGS% ./py_cc/crc8.py
wsl %MPY_CROSS% %MPY_CROSS_ARGS% ./py_cc/cc_constants.py
wsl %MPY_CROSS% %MPY_CROSS_ARGS% ./py_cc/cc_classes.py
wsl %MPY_CROSS% %MPY_CROSS_ARGS% ./py_cc/cc_protocol.py

wsl %MPY_CROSS% %MPY_CROSS_ARGS% ./utils/serial.py
wsl %MPY_CROSS% %MPY_CROSS_ARGS% ./utils/thread.py

rshell -p %COM_PORT% -f copy_firmware.rshell

pause