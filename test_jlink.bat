@echo off
echo Testing J-Link connection...
"C:\Program Files\SEGGER\JLink_V874a\JLink.exe" -device STM32G474VC -if SWD -speed 4000 -autoconnect 1 -CommanderScript test_jlink_commands.txt
pause
