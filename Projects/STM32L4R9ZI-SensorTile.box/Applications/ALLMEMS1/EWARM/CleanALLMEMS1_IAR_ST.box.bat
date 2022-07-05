@echo off
set STLINK_PATH="C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\"
set NAMEALLMEMS2=STM32L4R9ZI_SensorTilebox\Exe\STM32L4R9ZI-SensorTile.box_ALLMEMS1
set BOOTLOADER="..\..\..\..\..\Utilities\BootLoader\STM32L4R9ZI\BootLoaderL4R9.bin"
color 0F
echo                /******************************************/
echo                          Clean FP-SNS-ALLMEMS1
echo                /******************************************/
echo                              Full Chip Erase
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c UR -HardRst -ME
echo                /******************************************/
echo                              Install BootLoader
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -P %BOOTLOADER% 0x08000000 -V "after_programming"
echo                /******************************************/
echo                          Install FP-SNS-ALLMEMS1
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -P %NAMEALLMEMS2%.bin 0x08004000 -V "after_programming"
echo                /******************************************/
echo                     Dump FP-SNS-ALLMEMS1 + BootLoader
echo                /******************************************/
set offset_size=0x4000
for %%I in (%NAMEALLMEMS2%.bin) do set application_size=%%~zI
echo %NAMEALLMEMS2%.bin size is %application_size% bytes
set /a size=%offset_size%+%application_size%
echo Dumping %offset_size% + %application_size% = %size% bytes ...
echo ..........................
%STLINK_PATH%ST-LINK_CLI.exe -Dump 0x08000000 %size% %NAMEALLMEMS2%_BL.bin
echo                /******************************************/
echo                                 Reset STM32
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c UR -Rst
if NOT "%1" == "SILENT" pause