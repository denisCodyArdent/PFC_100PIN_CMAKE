@echo off
echo ========================================
echo STM32G474 Option Bytes Configuration
echo Using STM32CubeProgrammer
echo ========================================
echo.

REM Adjust this path if STM32CubeProgrammer is installed elsewhere
set STM32_PROG_PATH=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin

if not exist "%STM32_PROG_PATH%\STM32_Programmer_CLI.exe" (
    echo ERROR: STM32CubeProgrammer not found!
    echo Please install it or use the JLink script instead.
    pause
    exit /b 1
)

echo Connecting to STM32G474VC...
"%STM32_PROG_PATH%\STM32_Programmer_CLI.exe" -c port=SWD -ob nBOOT_SEL=1 nBOOT0=1

echo.
echo ========================================
echo Configuration complete!
echo Device will now boot from Flash memory
echo ========================================
pause
