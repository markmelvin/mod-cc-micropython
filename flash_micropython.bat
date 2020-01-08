set ST_FLASH=..\..\stlink-1.3.0-win64\bin\st-flash.exe
set FIRMWARE0=..\micropython\ports\stm32\build-NUCLEO_F446RE\firmware0.bin
set FIRMWARE1=..\micropython\ports\stm32\build-NUCLEO_F446RE\firmware1.bin

rem Flash main micropython binary to device
%ST_FLASH% write %FIRMWARE0% 0x08000000
%ST_FLASH% --reset write %FIRMWARE1% 0x08020000

pause