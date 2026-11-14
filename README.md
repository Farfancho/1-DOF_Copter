# 1-DOF Copter - STM32 Control

A one-degree-of-freedom propeller-driven test bench for experimenting with system identification, discrete-time feedback control, and embedded motor actuation. A brushless DC motor generates thrust to rotate a constrained arm. the angle is measured though a linear potentiometer and captured by a STM32 Nucleo board via ADC, the Nucleo computes the PID algorithm and drives the ESC with a PWM output. The system accepts commands through serial UART communication.

This repo includes the **STM32 Firmware**, **mechanical design files**, and a **ROS2 Integration**

> **Project Status:**  
    - STM32 firmware is implemented to receive a serial command, compute the PID algorithm and output a PWM signal to the ESC.  
    - ROS2 bridge is still in work progress and is not yet compatible with the STM32 Firmware.

## Features

- Open-loop ESC control and closed-loop angular position control.
- Difference equation based controller with configurable proportional, integral and derivative gains.
- Timer-triggered ADC sampling at 50Hz.
- PWM generation for a RC-style ESC.
- Runtime commands for control mode, angle setpoints, PID gains and stop.
- binary serial telemetry for plotting angle, control effort, error and setpoint.

## Hardware
  
| Component | Role |
| --- | --- |
| STM32 Nucleo-F446RE | Signal acquisition, control computation, PWM, and serial communication |
| Drone brushless DC motor and electronic speed controller (ESC) | Generate the thrust require to move the arm |
| Single-axis rotating arm | Constrains motion to one degree of freedom |
| Rotary potentiometer | Measures arm angle through a voltage divider |
| External motor power supply | Powers the motor through the ESC |
| USB/ST-LINK connection | Firmware programming and serial communication |

### Firmware pin assignment

| Signal | STM32 pin / peripheral |
| --- | --- |
| Potentiometer output | `PA0` — `ADC1_IN0` |
| ESC PWM command | `PA15` — `TIM2_CH1` |
| Serial TX / RX | `PA2` / `PA3` — `USART2` |
| ADC trigger / control timing | `TIM3` update event |

## How it works

STM32 uses `TIM3` to trigger an ADC conversion every 20 milliseconds. On it's completion callback, the position error is computed and if closed-loop operation is enabled, the controller calculates the PWM output for the ESC. on the other hand if the open-loop operation is enabled, the PWM output is written directly from the serial command.

The closed loop controller uses the difference equation:

$$
    u[k] = c_1u[k-1] + c_2u[k-2] + c_3e[k] + c_4e[k-1] + c_5e[k-2]
$$

where `e[k]` is the position error and `u[k]` is the controller output. The coefficients can be recomputed using `Kp`, `Ki` and `Kd`, This difference equation approach allows using conventional PID gains while keeping the advantages of the discrete PID such as using only past outputs and errors to determine the current outputs.

**Current implementation details:**

- Angle mapping in the firmware: ADC readings `125`–`630` correspond approximately to `45°`–`90°`. Recalibrate this mapping for your potentiometer and mechanical mounting.
- Default reference initialized in `main.c`: `80°`.
- General PWM mapping: `90000 + 90 × command`, with the command limited to `0`–`999` by `WritePWM()`; this corresponds to approximately 1–2 ms pulses at 50 Hz.
- Closed-loop controller output is additionally limited to `0`–`400` in `main.c`. Open-loop commands use the general `0`–`999` mapping.


## Getting started

### 1. Clone the repository

```bash
git clone https://github.com/Farfancho/1-DOF_Copter.git
cd 1-DOF_Copter
```

### 2. Build and flash the firmware

1. Open **STM32CubeIDE** and import the project in [`1-DOF_Copter/`](1-DOF_Copter/).
2. Review the `.ioc` peripheral configuration and physical wiring.
3. Build the firmware for the **NUCLEO-F446RE** board.
4. Connect the board through ST-LINK/USB and flash the program.
5. Connect to the board's serial port at **115200 baud, 8-N-1**.

> [!CAUTION]  
> Secure the mechanism, keep clear of the propeller. Disconnect motor power when changing wiring.

### 3. Send serial commands

Commands are ASCII strings terminated by newline (`\n`). For example, send `O200` followed by Enter to request an open-loop output of 200.

| Command | Example | Behavior |
| --- | --- | --- |
| `O<value>` | `O200` | Disable closed-loop mode and apply open-loop motor command |
| `C` | `C` | Enable closed-loop angular-position control |
| `R<angle>` | `R80` | Set the reference angle for closed loop operation |
| `P<value>` | `P1.5` | Update proportional gain and recalculate controller coefficients |
| `I<value>` | `I0.2` | Update integral gain and recalculate controller coefficients |
| `D<value>` | `D0` | Update derivative gain and recalculate controller coefficients |
| `S` | `S` | Disable closed-loop mode and command minimum PWM |

Tune the controller for your hardware and operating point before enabling closed-loop control. `S` commands minimum configured ESC PWM, it does not guarantee the removal of motor power.

### 4. Read telemetry

The firmware sends a **binary 8-byte frame**, approximately every 20 ms:

| Bytes | Format | Signal |
| --- | --- | --- |
| 0–1 | Unsigned 16-bit, little-endian | Measured angle |
| 2–3 | Unsigned 16-bit, little-endian | Control output |
| 4–5 | Unsigned 16-bit, little-endian | Error |
| 6–7 | Unsigned 16-bit, little-endian | Reference angle |

To properly visualize the variables, an offset is recommended to avoid error saturation over negative values, and to visualize the standard form of the PWM, a `0.1` gain is recommended to the control output.

This plotting configurations are included on the  [`Settings1DOF-Copter.ini`](Settings1DOF-Copter.ini) file, which is intended to work on the SerialPlot software. It contains channel names, offsets, and example commands. A plain-text serial monitor will not correctly display these binary frame; use a compatible plotting/decoding tool.

## ROS 2 workspace (experimental)
Coming soon...

## Repository structure

```text
.
├── 1-DOF_Copter/                # STM32 HAL firmware
│   ├── Core/Src/main.c          # Peripheral setup, sampling, mode handling
│   ├── Core/Src/control.c       # Control logic
│   ├── Core/Src/serial.c        # UART commands and telemetry
│   └── 1-DOF_Copter.ioc         # STM32CubeMX peripheral configuration
├── Mecanico/                   # CAD archive, BOM, mechanical drawings
├── copter_ws/src/
│   ├── copter/                  # ROS 2 Python nodes
│   └── interface_copter/       # Custom ROS 2 message definition
├── Settings1DOF-Copter.ini     # Serial plotting configuration
└── LICENSE                     # Apache License 2.0
```

## Thoughs of the project and next steps

This project was made for the controls and systems class of my mechatronic's degree so the scope was on learning PID tuning and basic control techniques. I also took this project as an oportunity to get closer to the STM32 enviroment, which i think worked out great. I'm really excited to do similar work in a close future. stay tuned!