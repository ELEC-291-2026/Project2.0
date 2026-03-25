# ELEC291 Project 2 — Field Detection and Remote Control Robot
**University of British Columbia — Electrical and Computer Engineering**  
**Course:** ELEC291/ELEC292  
**Instructor:** Dr. Jesús Calviño-Fraga  

---

## Table of Contents
1. [Project Overview](#project-overview)
2. [System Architecture](#system-architecture)
3. [Hardware Requirements](#hardware-requirements)
4. [Robot Subsystems](#robot-subsystems)
5. [Remote Subsystems](#remote-subsystems)
6. [Wireless Communication](#wireless-communication)
7. [Playing Field Setup](#playing-field-setup)
8. [Operating Modes](#operating-modes)
9. [Parts List](#parts-list)
10. [Bonus Features](#bonus-features)

---

## Project Overview

This project involves designing, building, programming, and testing a robot that supports remote control and magnetic-wire field detection. The robot uses three LC tank field-detector circuits to sense the guide wire: two for left/right tracking and one for intersection detection. The system can operate in manual mode through the remote and can also run wire-following logic on the robot.

The system consists of two separate microcontroller-based units — a **robot** and a **remote controller** — that communicate wirelessly via JDY-40 radio modules. Both units must use microcontrollers from **different families** and be fully **battery powered**.

---

## System Architecture

### Robot
```
[4xAA Battery 6V]
        |
   [On/Off Switch]
        |
   [Power Regulation] ──── 3.3V ──── [Robot Microcontroller]
        |                                       |
        |                     ┌─────────────────┼──────────────────┐
        |                     |                 |                  |
   [6V Motor Rail]        [JDY-40]      [Field Detector x3]
        |                  (UART)             (ADC x3)
   [H-Bridge 1] ── [Motor L]
   [H-Bridge 2] ── [Motor R]
  (via Optocouplers)
```

### Remote
```
[9V Battery]
      |
 [Power Regulation] ──── 3.3V/5V ──── [Remote Microcontroller]
                                                |
                         ┌──────────────────────┼─────────────────┐
                         |                      |                 |
                      [JDY-40]            [LCD Display]      [Joystick]
                       (UART)           (6-pin parallel)      (ADC x2)
                                               |
                                          [Speaker]
                                           (PWM)
```

---

## Hardware Requirements

- Two microcontroller systems from **different families** — choose from:
  - NXP ARM — LPC824
  - ST Microelectronics ARM — STM32L051
  - Microchip PIC32 — MX130F064B
  - TI MSP430 — MSP430G2553
  - Atmel AVR — ATMega328P
  - Nuvoton 8051 — N76E003
  - Silicon Labs 8051 — EFM8LB12F64
- Both robot and remote must be **battery powered** (batteries not provided — buy your own)
- Discrete **MOSFET H-bridge** drivers (P-MOSFET + N-MOSFET + LTV-847 optocouplers)
- **Three LC tank field detector circuits** using ferrite inductors
- **JDY-40** radio modules (one per unit)
- Remote must have: **LCD display**, **speaker**, **joystick**
- All code written in **C** (no Arduino environment)

---

## Robot Subsystems

### Power Regulation
- 4×AA batteries (6V) power the motors directly via the H-bridge
- LM7805 regulates down to 5V for logic
- MCP1700 regulates down to 3.3V for microcontroller and JDY-40
- MBR150 diodes used for power source switching (USB vs battery)
- Add decoupling capacitors per LM7805 and MCP1700 datasheets
- **Do NOT connect the ground of the motor circuit to the ground of the logic circuit** — use optocouplers for isolation

### H-Bridge Motor Driver
- One H-bridge per motor (two total)
- Uses one P-MOSFET and one N-MOSFET per side
- LTV-847 optocouplers isolate microcontroller signals from motor power rail
- RD = 1kΩ, RT = 10kΩ (per the provided design)
- Microcontroller drives 4 PWM output pins total (2 per motor)
- CW rotation: top P-MOSFET and bottom N-MOSFET on opposite sides conduct
- CCW rotation: opposite pair conducts

### Field Detector Array
- Three detector circuits are used:
  - left tracker
  - right tracker
  - intersection detector
- Each circuit is: pickup inductor → amplifier → peak detector → ADC input
- The left and right channels are compared to keep the robot centered over the magnetic guide wire
- The third channel is used to detect intersections for preconfigured path decisions
- ADC reads all three channels continuously during wire-following tests
- Thresholds and filtering are tuned in firmware to match the analog amplifier gain

---

## Remote Subsystems

### Power Regulation
- 9V battery → LM7805 → 5V for LCD and logic
- MCP1700 → 3.3V for JDY-40
- Add decoupling capacitors

### LCD Display
- 6 digital output pins from microcontroller
- Parallel interface, bit-banged
- Displays: current mode (manual/auto), battery level, signal status

### Joystick
- Two potentiometers (X and Y axes) → 2 ADC inputs on microcontroller
- **Note:** Joystick pins are not breadboard compatible out of the box — solder short hookup wires to pins, bend perpendicular, then plug into breadboard
- ADC values mapped to motor direction and speed commands

### Speaker
- Single PWM output pin
- Generates tones for: mode switch, field/intersection event, low battery warning

---

## Wireless Communication

### JDY-40 Radio Module
- Operating voltage: **3.3V only** — double check polarity before powering
- Interface: UART (TXD, RXD, SET) — 3 pins
- Maximum baud rate: **9600 baud**
- Pin pitch is 2mm — breadboard holes are 2.54mm:
  1. Bend header pins to match 2mm spacing
  2. Solder header on top of JDY-40
  3. Plug into breadboard
- **Set a unique device ID** for your pair to avoid interference with other groups:
```c
  SendATCommand("AT+DVIDxxxx\r\n"); // xxxx = 0000 to FFFF in hex
```
- JDY-40 should have a dedicated hardware UART if available
- For micros without a spare hardware UART (ATMega328p, MSP430G2553): use timer ISR software UART — examples on Canvas

### Command Protocol
Define a simple byte-based command set, for example:
```
0x01 — Forward
0x02 — Backward
0x03 — Left
0x04 — Right
0x05 — Stop
0x06 — Auto mode on
0x07 — Auto mode off
0x08 — Field/intersection status (robot → remote)
```

### JDY-40 Wiring by Microcontroller
| Microcontroller | TXD | RXD | SET |
|----------------|-----|-----|-----|
| EFM8 | P0.0 | P0.1 | P2.0 |
| ATMega328P | PB2 | PB1 | PD4 |
| MSP430G2553 | P2.1 | P2.2 | P2.0 |
| LPC824 | PIO0-9 | PIO0-8 | PIO0-1 |
| STM32L051 | PA14 | PA13 | PA15 |
| PIC32MX130 | B15 | B14 | B13 |

Add 1kΩ series resistors on all JDY-40 signal lines for protection.

---

## Playing Field Setup

- Minimum area: **0.5 m²**
- Recommended surface: Con-Tact non-adhesive shelf liner (clear diamonds, 60"×20") from Home Depot (~$17) — good grip, easy to roll and transport, fits lab benches
- Magnetic guide wire laid out on the playing field
- Intersections are formed by the wire layout
- Signal source drives the guide wire during field-following tests

---

## Operating Modes

### Manual Mode
- Remote joystick directly controls robot movement
- Joystick X/Y ADC values → direction and speed commands → transmitted via JDY-40 → robot executes motor control
- Speaker and LCD on remote provide feedback

### Automatic Mode
- Robot follows the magnetic guide wire using the three field detectors
- State machine:
  1. **Follow** — compare left and right detector signals and steer to stay centered
  2. **Intersection** — detect a new intersection with the third detector
  3. **Action** — move straight, left, right, or stop using the selected preconfigured path
  4. **Lost** — stop when the guide wire signal disappears
- Remote can start/stop auto mode and display live status


---
### Important Notes
- **Do not copy/paste block diagrams from the project spec** — redraw your own, it takes 5 minutes in Word
- Do not append datasheets or compiler manuals
- Document circuit values, ADC thresholds, and PWM settings in real time — don't try to reconstruct them after the fact
- The detailed design section is where most of the report marks come from — give every subsystem a proper circuit diagram and written explanation

---

## Parts List

### Provided in Kit (~$90 from ECE Stores, MCLD1032)
| Part | Description | Qty |
|------|-------------|-----|
| Solarbotics GM4 | Gear motors | 2 |
| 3D printed wheels | 70mm × 7mm elastic band wheels | 1 pair |
| Tamiya 70144 | Ball caster | 1 |
| Aluminum chassis | Water jet cut at UBC | 1 |
| 4×AA battery holder | Robot motor power | 1 |
| 9V battery clip | Remote or perimeter source | 1 |
| 1mH inductors | Ferrite core wirewound (DigiKey M8275-ND) — for the three field detector circuits | 3 |
| LTV-847 | Quad optocoupler | 2 |
| P-MOSFET + N-MOSFET | H-bridge transistors | per H-bridge |
| JDY-40 | Wireless radio module | 2 |
| MCP1700 | 3.3V regulator | 2 |
| LM7805 | 5V regulator | 2 |
| MBR150 | Schottky diodes for power switching | 4 |
| #28 magnet wire | For perimeter wire | 1 roll |

### Buy Yourself
| Part | Approx Cost | Notes |
|------|-------------|-------|
| 4×AA batteries | ~$5 | Brand name recommended — lower internal resistance |
| 9V batteries (x2) | ~$5 | One for remote, one for 555 timer perimeter source |
| Con-Tact shelf liner | ~$17 | Playing field surface — Home Depot |

---

## Bonus Features

These features go beyond the base requirements and are what push a demo score from 7.5 into the 8–10 range. All are achievable within the project timeline and all work with a standard breadboard setup. Total hardware cost for all of them is around $29, or roughly $5 per person.

---

### HC-SR04 Ultrasonic Obstacle Avoidance
**What it does:** Detects objects in the robot's path and stops or steers around them in auto mode.

**Hardware:** HC-SR04 sensor module (~$4 on Amazon). 4-pin, 2.54mm pitch, plugs directly into breadboard. Runs on 5V.

**Software:** Two GPIO pins — one trigger, one echo. Send a 10µs pulse on trigger, measure the pulse width on echo. Distance = pulse width × speed of sound / 2. Stop or turn when distance drops below threshold.

**Difficulty:** 2/5 | **Time:** 3–4 hours

**Why it's worth it:** Adds genuine autonomous functionality. Directly addresses the "additional functionality" criterion in the marking rubric.


---

### ESP32-CAM Live FPV Stream
**What it does:** Mounts a small camera on the robot and streams live video over WiFi. Anyone in the room can open a browser on their phone and watch the robot's perspective in real time during the demo.

**Hardware:** ESP32-CAM module + programmer board (~$10 on Amazon, usually sold as a bundle). Solder 4 jumper wires to connect to breadboard power rails.

**Software:** Flash the pre-built CameraWebServer firmware — built into Arduino IDE under File → Examples → ESP32 → Camera → CameraWebServer. Change two lines (WiFi SSID and password). Flash it. Done. No custom code required.


---

### Web Telemetry Dashboard (Wemos D1 Mini)
**What it does:** Robot broadcasts live telemetry to a webpage — battery voltage, current mode, field sensor readings, and motor speeds — accessible by anyone on the same WiFi network.

**Hardware:** Wemos D1 Mini ESP8266 (~$5 on Amazon). **Buy the D1 Mini version, not the bare ESP-01 module.** Standard 2.54mm headers, plugs straight into breadboard. Connects to robot microcontroller via UART.

**Software:** D1 Mini runs a lightweight web server. Robot microcontroller sends telemetry packets over UART to the D1 Mini, which formats and serves them as a live-updating webpage. HTML/CSS/JavaScript for the dashboard frontend.


---

### Field Signal Logger
**What it does:** Robot logs left, right, and intersection detector strengths over time so you can tune thresholds and verify wire-following behavior.

**Hardware:** No additional hardware required.

**Software:** Stream or store filtered left/right/intersection values together with the current path state. Use the logs to adjust thresholds, confirm intersection detection, and debug lost-wire events.



*README last updated: March 2026*  
*ELEC291 Project 2 — Field Detection and Remote Control Robot — UBC ECE*
