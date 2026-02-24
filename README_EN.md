# 3RRS Parallel Platform Solar Tracking System

A solar tracking system based on a **3-RRS parallel mechanism**. MATLAB handles kinematics simulation and control command generation, which are transmitted via serial port to an STM32F103 microcontroller, driving a PCA9685 servo driver board to control three servos.

---

## System Architecture

```
MATLAB (PC)
  ├── sun.m       ← Pure simulation (no hardware required)
  └── sun211.m    ← Simulation + hardware control (serial output)
        │  UART 115200 baud
        │  Protocol: "S1:090\n"
        ▼
STM32F103 (Core/Src/main.c + usart.c)
        │  I2C
        ▼
PCA9685 Servo Driver (Core/Src/pca9685.c)
        │  50Hz PWM (500~2500 µs)
        ▼
3× Servo Motors (one per RRS chain)
```

---

## File Structure

```
tracker_MATLAB_STM32/
├── sun.m               # MATLAB pure simulation (no hardware dependency)
├── sun211.m            # MATLAB simulation + serial control
├── testmotor.ioc       # STM32CubeMX configuration file
├── Core/
│   ├── Inc/
│   │   ├── pca9685.h   # PCA9685 driver interface
│   │   ├── usart.h
│   │   └── i2c.h
│   └── Src/
│       ├── main.c      # STM32 main program, implements OnServoCommand()
│       ├── pca9685.c   # PCA9685 I2C driver (50Hz PWM output)
│       ├── usart.c     # UART interrupt receive & command parsing
│       └── i2c.c
├── MDK-ARM/
│   └── testmotor.uvprojx   # Keil MDK project file
└── Drivers/                # STM32 HAL library
```

---

## Mechanism Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `Rb` | 0.20 m | Base platform joint radius |
| `Rp` | 0.12 m | Moving platform joint radius |
| `h`  | 0.18 m | Initial platform height |
| `L1` | 0.10 m | Active arm length |
| `L2` | 0.18 m | Passive arm length |
| Max tilt | ±35° | Roll / Pitch direction |

---

## MATLAB Programs

### `sun.m` — Pure Simulation
- No hardware dependency; serial connection failure is silently skipped
- Use this to validate IK algorithms and PID parameters without hardware

### `sun211.m` — Simulation + Hardware Control
- Auto-detects available serial ports, or manually specify `portPref`
- Move the mouse in the **Control Pad** window to set the tracking direction
- Sends servo commands at 50Hz; only transmits when angle changes

**Three visualization windows:**

| Window | Content |
|--------|---------|
| Window 1 | 3RRS mechanism 3D animation + floating sun sphere |
| Window 2 | Mouse control pad (Roll/Pitch target input) |
| Window 3 | Real-time servo angle curves |

**Serial Protocol:**
```
S1:090   ← Servo 1, 90°
S2:045   ← Servo 2, 45°
S3:120   ← Servo 3, 120°
```
Each command is terminated with `\n`. Angle is always zero-padded to 3 digits.

---

## STM32 Firmware

### Hardware Configuration
| Peripheral | Setting |
|------------|---------|
| USART1 | 115200 baud, 8N1 |
| I2C1 | Connected to PCA9685 (address 0x40) |
| PCA9685 PWM | 50Hz, channels 0/1/2 → servo 1/2/3 |

### Command Parsing (`usart.c`)
- Byte-by-byte interrupt reception; parses on `\n` or `\r`
- Format validation: `S[1-3]:[0-9]{3}`
- Calls `OnServoCommand(id, angle)` to drive the corresponding servo

### PCA9685 Driver (`pca9685.c`)
- `PCA9685_Init50Hz()` — Initialize at 50Hz PWM
- `PCA9685_ServoWriteDeg(addr, ch, deg)` — Convert angle to pulse width
- Pulse width range: 500 µs (0°) ~ 2500 µs (180°)

---

## Quick Start

### 1. Simulation Only (No Hardware)
```matlab
run('sun.m')
```
Move the mouse in the Control Pad window to see the 3RRS mechanism follow.

### 2. With Hardware
1. Build and flash the STM32 firmware (`MDK-ARM/testmotor.uvprojx` in Keil)
2. Connect the STM32 USB-serial adapter and note the COM port
3. Edit `sun211.m` (or leave blank for auto-detection):
   ```matlab
   portPref = "COM5";  % Set your actual COM port, or "" for auto
   ```
4. Run in MATLAB:
   ```matlab
   run('sun211.m')
   ```

### 3. Servo Calibration
If servos don't center correctly, adjust the mapping in `sun211.m`:
```matlab
servoOffset = [90 90 90];   % Neutral angle for each servo
servoDir    = [-1 -1 -1];   % Direction: 1 or -1
```

---

## Requirements

| Side | Requirement |
|------|-------------|
| MATLAB | R2019b or later (requires `serialport` function) |
| Keil MDK | V5, with STM32F1xx HAL library |
| Hardware | STM32F103 board + PCA9685 module + 3× servo motors |

---

## PID Tuning Reference

```matlab
Kp = 0.02;   % Proportional: increase for faster response
Ki = 0;      % Integral: keep at 0 for most cases
Kd = 0;      % Derivative: increase slightly to reduce oscillation
```
