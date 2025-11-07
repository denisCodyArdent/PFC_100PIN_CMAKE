# GDB script to debug SystemClock_Config issues
# This script will help identify why the system stops at SystemClock_Config

echo \n========================================\n
echo  Clock Configuration Debug Script\n
echo ========================================\n\n

# Set breakpoints at critical locations
echo Setting breakpoints...\n
break main
break SystemClock_Config
break HAL_RCC_OscConfig
break HAL_RCC_ClockConfig
break Error_Handler
break HardFault_Handler
break NMI_Handler

# Monitor RCC register changes
echo \nConfiguring watchpoints for RCC registers...\n

# Commands to run when hitting SystemClock_Config
commands 2
  echo \n*** Entered SystemClock_Config ***\n
  echo Current RCC_CR register (0x40021000):\n
  x/1xw 0x40021000
  echo Current RCC_CFGR register (0x40021008):\n
  x/1xw 0x40021008
  echo Current RCC_PLLCFGR register (0x4002100C):\n
  x/1xw 0x4002100C
  echo \n
  continue
end

# Commands for HAL_RCC_OscConfig
commands 3
  echo \n*** Entered HAL_RCC_OscConfig ***\n
  echo Checking RCC_CR for HSE status:\n
  echo RCC_CR (0x40021000):\n
  x/1xw 0x40021000
  printf "  Bit 16 (HSERDY): %d (1=HSE ready, 0=not ready)\n", (*(unsigned int*)0x40021000 >> 17) & 1
  printf "  Bit 17 (HSEON): %d (1=HSE enabled)\n", (*(unsigned int*)0x40021000 >> 16) & 1
  echo \n
  continue
end

# Commands for Error_Handler
commands 5
  echo \n*** ERROR: Entered Error_Handler! ***\n
  echo This means the clock configuration failed.\n
  echo \nRCC Status Registers:\n
  echo RCC_CR (0x40021000):\n
  x/1xw 0x40021000
  printf "  HSION (bit 8): %d\n", (*(unsigned int*)0x40021000 >> 8) & 1
  printf "  HSIRDY (bit 10): %d\n", (*(unsigned int*)0x40021000 >> 10) & 1
  printf "  HSEON (bit 16): %d\n", (*(unsigned int*)0x40021000 >> 16) & 1
  printf "  HSERDY (bit 17): %d (1=HSE READY, 0=HSE NOT READY)\n", (*(unsigned int*)0x40021000 >> 17) & 1
  printf "  PLLON (bit 24): %d\n", (*(unsigned int*)0x40021000 >> 24) & 1
  printf "  PLLRDY (bit 25): %d\n", (*(unsigned int*)0x40021000 >> 25) & 1
  echo \nRCC_CFGR (0x40021008):\n
  x/1xw 0x40021008
  echo \nRCC_PLLCFGR (0x4002100C):\n
  x/1xw 0x4002100C
  echo \nBacktrace:\n
  backtrace
  echo \n*** Stopped at Error_Handler - Check if HSERDY=0 (HSE failed) ***\n
end

# Commands for HardFault
commands 6
  echo \n*** HARDFAULT OCCURRED! ***\n
  echo Fault Status Registers:\n
  echo HFSR (HardFault Status Register):\n
  x/1xw 0xE000ED2C
  echo CFSR (Configurable Fault Status Register):\n
  x/1xw 0xE000ED28
  echo MMFAR (MemManage Fault Address):\n
  x/1xw 0xE000ED34
  echo BFAR (BusFault Address):\n
  x/1xw 0xE000ED38
  echo \nStack frame:\n
  backtrace
  echo \nRegisters:\n
  info registers
end

echo \nBreakpoints set. Starting execution...\n
echo \n========================================\n\n

# Start execution
continue

# If we get here, print final status
echo \n========================================\n
echo  Final Debug Status\n
echo ========================================\n
echo Current location:\n
backtrace
echo \nRCC_CR:\n
x/1xw 0x40021000
echo \nRCC_CFGR:\n
x/1xw 0x40021008
