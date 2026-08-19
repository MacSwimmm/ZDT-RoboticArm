# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for a **2-axis auto-tracking gimbal** (二维云台) on an **STM32F407VETx** (Cortex-M4F, 168 MHz). A MaixCAM vision module streams the tracked target's pixel coordinates over UART; the MCU runs per-axis PID and drives two **ZDT (张大头) closed-loop stepper motors** in velocity mode so the camera keeps the target centered.

```
MaixCAM --USART2--> Camera_Process --> Gimbal_UpdateTarget
                    --> Gimbal_Loop (PID, 20ms) --> Emm_V5 velocity cmd
                    --USART1--> 2x ZDT stepper drivers
```

## Start here

A detailed CLAUDE.md lives in the Keil project folder and is the authoritative guide for this firmware: **[MDK-ARM/CLAUDE.md](MDK-ARM/CLAUDE.md)**. It covers:

- The Keil MDK-ARM build/flash workflow (no Makefile/CMake; UV4 headless commands, `build_output.log`).
- The CubeMX-owned vs. hand-written code split (`Core/`, `Drivers/` vs. `MDK-ARM/APP`, `Algorithm`, `Hardware`).
- Per-file architecture (`Camera.c`, `Gimbal_Control.c`, `PID.c`, `Emm_V5.c`).
- The `g_emm_debug` struct for diagnosing USART1 DMA lockups.
- The current **bring-up debug loop in `main.c`** (around line 162) that blocks the tracking app from running — read this section before flashing.

Read that file before making non-trivial changes. The notes below are the minimal subset needed to orient.

## Layout (one-screen map)

| Path | Owner | Notes |
| --- | --- | --- |
| `F407-AutoTracking-Gimbal.ioc` | CubeMX | Pinout / clock / peripheral config. Edit in STM32CubeMX. |
| `Core/Inc/`, `Core/Src/` | CubeMX | HAL + main. **Edit only inside `/* USER CODE BEGIN/END */` guards** — regeneration overwrites the rest. |
| `Drivers/STM32F4xx_HAL_Driver/`, `Drivers/CMSIS/` | ST | Vendored HAL/CMSIS. Do not hand-edit. |
| `MDK-ARM/F407-AutoTracking-Gimbal.uvprojx` | Keil | Project file (ARMCC V5.06). Add new `APP/Algorithm/Hardware` `.c` files here, and to `.vscode/c_cpp_properties.json` includePath for IntelliSense. |
| `MDK-ARM/Hardware/Camera.c` | hand-written | USART2 (MaixCAM) receive + parse. |
| `MDK-ARM/APP/Gimbal_Control.c` | hand-written | Orchestrator + per-axis PID + motor command dispatch. **All user-tunable constants live as `#define`s at the top.** |
| `MDK-ARM/Algorithm/PID.c` | hand-written | Position-form PID; output unit is **RPM**. |
| `MDK-ARM/Algorithm/Emm_V5.c/.h` | vendor (张大头) | ZDT stepper protocol. Every send goes through `HAL_UART_Transmit_DMA` on `huart1`, wrapped to record into `g_emm_debug`. |

## Build / flash (cheat sheet)

```bash
# Build (overwrites build_output.log)
"C:/Keil_v5/UV4/UV4.exe" -b MDK-ARM/F407-AutoTracking-Gimbal.uvprojx -o MDK-ARM/build_output.log

# Rebuild all
"C:/Keil_v5/UV4/UV4.exe" -r MDK-ARM/F407-AutoTracking-Gimbal.uvprojx -o MDK-ARM/build_output.log
```

UV4 returns immediately; always check the log. A clean build ends with `0 Error(s)`. Output: `MDK-ARM/F407-AutoTracking-Gimbal/F407-AutoTracking-Gimbal.axf` (+ `.hex`). Flash via Keil/ST-Link. **There are no automated tests** — verification is flashing and observing the gimbal / `g_emm_debug` in a debugger.

## Fixed invariants — do not change without good reason

- **UART assignment**: USART1 = motor bus (115200 8N1), USART2 = MaixCAM (115200 8N1). Callbacks in `Camera.c` filter by `huart->Instance` to enforce this.
- **Motor addressing is set inside the driver, not on MCU pins.** X = address 1, Y = address 2. New ZDT drivers default to address 1 — re-address one at a time before paralleling on the bus.
- **All motor commands must be wrapped by `MotorBus_WaitTxReady()`** in `Gimbal_Control.c`; back-to-back sends otherwise hit `HAL_BUSY` and drop.
- **Do not enable the drivers' auto/position-reached reply on both motors** — multiple devices replying on the shared TTL bus collide. The firmware drains replies into a throwaway buffer; it does not parse them.
- **PID output unit is RPM** (deliberate — supersedes the older C8T6 pulse-frequency design referenced in comments).

## Reference material (outside this project)

The parent folder (`../`) holds sibling projects used as references — `C8T6-Tracking_Gimbal基础HAL库配置` (the older F103 pulse-based design this code evolved from), the `STM32F407_CAN/串口通讯__多机同步控制` multi-motor sync examples, and ZDT_XS driver documentation. Comments in this codebase that say "旧 F103 工程" refer to that C8T6 project.
