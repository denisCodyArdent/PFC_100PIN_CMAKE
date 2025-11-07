@echo off
echo Testing J-Link with detailed commands...
echo.
"C:\Program Files\SEGGER\JLink_V874a\JLink.exe" -device STM32G474VC -if SWD -speed 4000 -autoconnect 1 -CommandFile test_jlink_detailed.jlink
echo.
echo Test complete. Check output above for any errors.
pause
