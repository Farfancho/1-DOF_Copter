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
