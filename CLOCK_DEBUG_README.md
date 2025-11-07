# Clock Configuration Debug Guide

## Problem Summary
The firmware is stopping at `SystemClock_Config()` function in [main.c](Core/Src/main.c#L146).

## Root Cause Analysis

### Most Likely Issue: **HSE Oscillator Failure**

Your code is configured to use a **24 MHz external HSE oscillator** at [main.c:158-159](Core/Src/main.c#L158-L159), but if:
- No external crystal is physically present on the board
- The crystal is not oscillating properly
- Wrong crystal frequency
- Poor PCB layout or capacitor values

Then `HAL_RCC_OscConfig()` will **timeout waiting for HSERDY flag** and call `Error_Handler()` at [main.c:169](Core/Src/main.c#L169), which enters an infinite loop.

### Configuration Details

**Current Clock Setup:**
- **Clock Source**: HSE (External 24 MHz crystal)
- **PLL Configuration**:
  - PLLM = 4 (divider)
  - PLLN = 50 (multiplier)
  - PLLR = 2 (divider)
  - Target SYSCLK = (24MHz / 4) × 50 / 2 = **150 MHz**
- **Flash Latency**: 4 wait states (correct for 150MHz)

**Expected Hardware**: 24 MHz crystal on pins PF0 (OSC_IN) and PF1 (OSC_OUT)

## Diagnostic Tools Created

### 1. JLink Hardware Diagnostic Script
**File**: [diagnose_clock.jlink](diagnose_clock.jlink)

**Usage**:
```batch
"C:\Program Files\SEGGER\JLink_V874a\JLink.exe" -device STM32G474VC -if SWD -speed 4000 -autoconnect 1 -CommandFile diagnose_clock.jlink
```

**What it checks**:
- RCC_CR register - shows HSE/HSI/PLL ready flags
- RCC_CFGR - current clock configuration
- RCC_PLLCFGR - PLL settings
- RCC_CIFR - Clock Security System flags
- Flash registers
- Option bytes
- Fault status registers
- Current execution state (PC, SP, LR)

**Key indicator**: If `HSERDY` (bit 17 in RCC_CR) = 0, the HSE failed to start.

### 2. GDB Debug Script
**File**: [debug_clock_issue.gdb](debug_clock_issue.gdb)

**Usage in VSCode**: Modify your launch.json to add:
```json
"postLaunchCommands": [
    "source debug_clock_issue.gdb"
]
```

Or run manually in GDB:
```
(gdb) source debug_clock_issue.gdb
```

**What it does**:
- Sets breakpoints at all critical clock configuration functions
- Monitors RCC registers at each step
- Catches entry into `Error_Handler()` and displays full diagnostic info
- Catches hard faults with fault status register dump

## Troubleshooting Steps

### Step 1: Run JLink Diagnostic
```batch
cd C:\Repo\PFC_100PIN_CMAKE
"C:\Program Files\SEGGER\JLink_V874a\JLink.exe" -device STM32G474VC -if SWD -speed 4000 -autoconnect 1 -CommandFile diagnose_clock.jlink
```

**Look for**:
- `RCC_CR` register bit 17 (HSERDY) - should be 1 if HSE is working
- `RCC_CIFR` register bit 3 (CSSF) - should be 0 (no clock failure)

### Step 2: Check Hardware
1. **Verify external crystal presence**: Check if your STM32G474VC board has a 24 MHz crystal
2. **Measure oscillator pins**: Use oscilloscope on PF0/PF1
3. **Check schematic**: Verify crystal + load capacitors (typically 2x 20pF for 24MHz)

### Step 3: Try HSI as Temporary Fix

If no external crystal exists, switch to **internal HSI oscillator** (16 MHz):

**Modify** [main.c:158-163](Core/Src/main.c#L158-L163):

```c
// OLD CODE (uses HSE):
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState = RCC_HSE_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
RCC_OscInitStruct.PLL.PLLN = 50;

// NEW CODE (uses HSI):
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
RCC_OscInitStruct.HSIState = RCC_HSI_ON;
RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
RCC_OscInitStruct.PLL.PLLN = 85;  // Changed to hit 170MHz: (16/4)*85/2=170MHz
```

This achieves **170 MHz** (max for STM32G4) using HSI.

### Step 4: Debug with GDB

Run your debug configuration and watch the console output. The script will automatically:
1. Show you when it enters `SystemClock_Config()`
2. Display RCC register values at each step
3. **Trap `Error_Handler()` with full diagnostic dump**
4. Show where and why it failed

### Step 5: Check for Flash Protection

Your repo has option byte scripts. Run:
```batch
cd C:\Repo\PFC_100PIN_CMAKE
"C:\Program Files\SEGGER\JLink_V874a\JLink.exe" -device STM32G474VC -if SWD -speed 4000 -autoconnect 1 -CommandFile read_option_bytes.jlink
```

Verify:
- RDP (Read Protection) = 0xAA (Level 0 - no protection)
- nBOOT_SEL = 1 (ignore BOOT0 pin)
- nBOOT0 = 1 (boot from Flash)

## Quick Reference: STM32G4 Clock Sources

| Source | Frequency | Accuracy | Use Case |
|--------|-----------|----------|----------|
| **HSI** | 16 MHz | ±1% | No external crystal needed |
| **HSE** | 4-48 MHz | ±50ppm | High precision (requires external crystal) |
| **LSI** | 32 kHz | ±5% | Low-power, RTC |
| **LSE** | 32.768 kHz | ±20ppm | Precise RTC (requires external crystal) |

## Common Error Patterns

### Pattern 1: Stops at SystemClock_Config, enters Error_Handler
**Cause**: HSE timeout (HSERDY never set)
**Solution**: Check HSE hardware or switch to HSI

### Pattern 2: Hardfault immediately after SystemClock_Config
**Cause**: Flash latency not set correctly for new frequency
**Solution**: Verify FLASH_LATENCY matches frequency (4 for 150-170MHz)

### Pattern 3: Code runs but clock frequency wrong
**Cause**: PLL multiplier/divider misconfiguration
**Solution**: Verify PLL calculations match target frequency

### Pattern 4: Watchdog reset during clock config
**Cause**: Clock config takes too long, IWDG triggers
**Solution**: Disable IWDG in option bytes or add IWDG refresh

## Register Reference

### RCC_CR (0x40021000) - Clock Control Register
```
Bit 17 HSERDY:  HSE ready flag (read-only)
Bit 16 HSEON:   HSE enable
Bit 10 HSIRDY:  HSI ready flag (read-only)
Bit 8  HSION:   HSI enable
Bit 25 PLLRDY:  PLL ready flag (read-only)
Bit 24 PLLON:   PLL enable
```

### RCC_CIFR (0x4002101C) - Clock Interrupt Flag Register
```
Bit 3 CSSF: Clock Security System failure flag
            (1 = HSE failed, CSS triggered)
```

### Expected Values (if HSE working)
- `RCC_CR` should show: `0x01XX7XXX` (HSERDY=1, HSEON=1)
- `RCC_CIFR` should show: `0x00000000` (no failures)

## Next Steps

1. **Run the JLink diagnostic**: Get the actual hardware state
2. **Share the output**: Post the RCC_CR value here
3. **Decide on fix**: HSE hardware repair OR switch to HSI

## Additional Resources

- [STM32G4 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) - Section 7 (RCC)
- [STM32G474 Datasheet](https://www.st.com/resource/en/datasheet/stm32g474vc.pdf) - Clock characteristics
- STM32CubeMX can regenerate correct clock config if you specify "no external crystal"

---

**Created**: Investigation of clock stoppage issue
**Files**: [main.c](Core/Src/main.c), [system_stm32g4xx.c](Core/Src/system_stm32g4xx.c)
