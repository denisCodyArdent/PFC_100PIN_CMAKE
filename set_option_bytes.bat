@echo off
echo ========================================
echo STM32G474 Option Bytes Configuration
echo Setting device to always boot from Flash
echo ========================================
echo.
"C:\Program Files\SEGGER\JLink_V874a\JLink.exe" -device STM32G474VC -if SWD -speed 4000 -autoconnect 1 -CommandFile set_option_bytes.jlink
echo.
echo ========================================
echo Configuration complete!
echo Device will now boot from Flash memory
echo ========================================
pause
