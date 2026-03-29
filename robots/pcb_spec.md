# Robot PCB / Perf Board Specification

**Date:** February 2026
**Author:** Kevin
**Status:** Draft — awaiting designer review

---

## 1. Project Overview

Two small autonomous robots that find and navigate toward each other using UWB (Ultra-Wideband) distance ranging, BLE (Bluetooth Low Energy), and an IMU for orientation. They need custom PCBs or perf boards to replace the current breadboard/flying-wire prototypes.

- **Robot-A (Emitter / UWB Tag)** — the seeker. Has an 8x8 LED matrix for directional arrows, an IR emitter LED, motors, IMU, and audio.
- **Robot-B (Receiver / UWB Anchor)** — the target. Has a single status LED, an IR phototransistor receiver, motors, IMU, and audio.

Both units are battery-powered (3.7V LiPo) and must be programmable via USB-C without disassembly.

---

## 2. MCU Options

Two MCU configurations are documented. The designer should advise on which is more practical for a compact board.

### Option A: Makerfabs MaUWB ESP32S3 (current prototype)

- https://www.makerfabs.com/mauwb-esp32s3-uwb-module.html
- **What it is:** Single board integrating ESP32-S3 + STM32 coprocessor + DW3000 UWB chip + 0.96" OLED (128x64)
- **Footprint:** ~60 x 25 mm (with headers)
- **UWB interface:** AT commands over internal UART (GPIO17/18), transparent to the designer
- **Available GPIOs for peripherals:** IO1 through IO11
- **Power input:** USB-C on-board, or VIN/GND pins
- **Advantages:** Proven working firmware, built-in OLED eliminates a separate display, AT command protocol avoids SPI complexity
- **Disadvantages:** Larger footprint, limited to 11 external GPIOs

### Option B: Seeed XIAO ESP32-S3 + Ai-Thinker BU03 DW3000 Module

- **XIAO ESP32-S3:** 21 x 17.8 mm, USB-C, built-in LiPo JST connector with charging circuit, 11 GPIOs (D0-D10)
- - https://grobotronics.com/seeed-studio-xiao-esp32-s3-plus.html
- **BU03 module:** 23 x 13 x 2.5 mm SMD-24 package, DW3000 UWB chip, onboard ceramic antenna, SPI interface
- https://openelab.io/products/ai-thinker-uwb-bu03-dw3000-plan-module-positioning-accuracy-10cm?variant=44538104807622&country=DE&currency=EUR
- **UWB interface:** SPI (SCK, MOSI, MISO, CS, IRQ, RST = 6 pins from XIAO). Requires new firmware driver — significant software change.
- **No built-in OLED.** An external 0.96" I2C OLED could be added if desired (shares I2C bus with BNO055).
- **Pin pressure:** BU03 SPI needs 6 pins. Combined with all other peripherals, pin assignment is tight. Designer should verify feasibility before committing.
- **Advantages:** Much smaller combined footprint, XIAO has built-in LiPo charging
- **Disadvantages:** Requires new UWB driver firmware, tight pin budget, no OLED unless added separately

---

## 3. Peripheral List

### 3.1 Shared Peripherals (both Robot-A and Robot-B)

| # | Peripheral | Model / Part | Interface | Power | Notes |
|---|---|---|---|---|---|
| 1 | IMU | DFRobot Gravity 10DOF (BNO055) | I2C, address 0x28 | 3.3V from MCU | Breakout board with JST cable. SDA/SCL/VCC/GND|
https://grobotronics.com/gravity-10-dof-imu-ahrs-bno055-bmp280.html
| 2 | Motor 1 | DFRobot Gravity DC Micro Metal Gear Motor 75:1 | Servo-style PWM, 1000-2000 us | VCC/GND direct from battery | Signal wire only from MCU |
https://grobotronics.com/gravity-dc-micro-metal-gear-motor-w-driver-75-1.html
| 3 | Motor 2 | DFRobot Gravity DC Micro Metal Gear Motor 75:1 | Servo-style PWM, 1000-2000 us | VCC/GND direct from battery | Signal wire only from MCU |
https://grobotronics.com/gravity-dc-micro-metal-gear-motor-w-driver-75-1.html
| 4 | Audio player | DFPlayer Mini MP3 | UART 9600 baud | 3.3V from MCU | TX line needs 1K ohm series resistor. RX direct. |
https://wiki.dfrobot.com/dfr0299
| 5 | Speaker | Small 8 ohm speaker | Wired to DFPlayer SPK1/SPK2 | From DFPlayer | **Mounted externally**, connected via 2-wire cable |
https://grobotronics.com/speaker-enclosed-2w-8ohm-20x30mm-ph1-25.html
| 6 | Battery | 3.7V LiPo, JST-PH 2-pin | JST connector | N/A | Robot-A: 2000 mAh. Robot-B: 1000 mAh. |
https://grobotronics.com/li-po-battery-3-7v-2000mah-molex-2-5mm.html

### 3.2 Robot-A (Emitter) Only

| # | Peripheral | Model / Part | Interface | Power | Notes |
|---|---|---|---|---|---|
| 7 | LED matrix | MAX7219 8x8 LED module | Bit-banged SPI (DIN, CLK, CS) | VCC/GND direct from battery | **Mounted externally**, connected via 5-wire cable |
https://grobotronics.com/led-matrix-8x8-red-with-max7219.html
| 8 | IR emitter | 940 nm IR LED | GPIO + 200 ohm series resistor | From GPIO | Mounted forward-facing on chassis, short wire to board |
https://www.hellasdigital.gr/electronics/sensors/infrared-sensors/940nm-ir-infrared-receiver-led/

### 3.3 Robot-B (Receiver) Only

| # | Peripheral | Model / Part | Interface | Power | Notes |
|---|---|---|---|---|---|
| 7 | Status LED | Standard visible LED (any color) | GPIO + 100 ohm series resistor | From GPIO | **Mounted externally**, 2-wire cable |
(to be provided)
| 8 | IR receiver | IR phototransistor (940 nm sensitive) | Analog input | 3.3V bias | Mounted forward-facing on chassis, short wire to board |
https://www.hellasdigital.gr/electronics/sensors/infrared-sensors/940nm-ir-infrared-receiver-led/
---

## 4. Pin Mapping

### 4.1 Option A — MaUWB ESP32S3

Internal pins (no routing needed — on-board):

| GPIO | Function | Notes |
|---|---|---|
| 16 | STM32 Reset | Directly connected on MaUWB PCB |
| 17 | STM32 AT UART TX | Internal UWB communication |
| 18 | STM32 AT UART RX | Internal UWB communication |
| 38 | OLED SCL | Internal I2C bus 0 |
| 39 | OLED SDA | Internal I2C bus 0 |

**Robot-A (Emitter) external wiring:**

| MaUWB IO | GPIO | Function | Routing |
|---|---|---|---|
| IO1 | 1 | DFPlayer TX | Through 1K resistor to DFPlayer RX pin |
| IO2 | 2 | DFPlayer RX | Direct to DFPlayer TX pin |
| IO3 | 3 | IR LED anode | Through 200 ohm resistor, cathode to GND |
| IO5 | 5 | BNO055 SDA | I2C bus 1 (Wire1) |
| IO6 | 6 | BNO055 SCL | I2C bus 1 (Wire1) |
| IO7 | 7 | Motor 1 signal | Direct to motor signal wire |
| IO8 | 8 | Motor 2 signal | Direct to motor signal wire |
| IO9 | 9 | MAX7219 DIN | To external header (matrix cable) |
| IO10 | 10 | MAX7219 CLK | To external header (matrix cable) |
| IO11 | 11 | MAX7219 CS | To external header (matrix cable) |

**Robot-B (Receiver) external wiring:**

| MaUWB IO | GPIO | Function | Routing |
|---|---|---|---|
| IO1 | 1 | DFPlayer TX | Through 1K resistor to DFPlayer RX pin |
| IO2 | 2 | DFPlayer RX | Direct to DFPlayer TX pin |
| IO3 | 3 | Status LED anode | Through 100 ohm resistor, cathode to GND |
| IO5 | 5 | BNO055 SDA | I2C bus 1 (Wire1) |
| IO6 | 6 | BNO055 SCL | I2C bus 1 (Wire1) |
| IO7 | 7 | Motor 1 signal | Direct to motor signal wire |
| IO8 | 8 | Motor 2 signal | Direct to motor signal wire |
| IO9 | 9 | IR phototransistor | Analog input, pull-down to GND as needed |

### 4.2 Option B — XIAO ESP32-S3 + BU03

**Robot-A (Emitter):**

| XIAO Pin | Function | Routing |
|---|---|---|
| D0 | MAX7219 DIN | To external header (matrix cable) |
| D1 | MAX7219 CLK | To external header (matrix cable) |
| D2 | MAX7219 CS | To external header (matrix cable) |
| D3 | BNO055 SDA | I2C (Wire) |
| D4 | Motor 1 signal | Direct to motor signal wire |
| D5 | BNO055 SCL | I2C (Wire) |
| D6 | Motor 2 signal | Direct to motor signal wire |
| D7 | BU03 SPI CS | To BU03 module |
| D8 | DFPlayer TX | Through 1K resistor to DFPlayer RX pin |
| D9 | DFPlayer RX | Direct to DFPlayer TX pin |
| D10 | IR LED anode | Through 200 ohm resistor, cathode to GND |
| SPI bus | BU03 SCK/MOSI/MISO/IRQ/RST | See note below |

**Robot-B (Receiver):**

| XIAO Pin | Function | Routing |
|---|---|---|
| D0 | Status LED anode | Through 100 ohm resistor, cathode to GND |
| D3 | BNO055 SDA | I2C (Wire) |
| D4 | Motor 1 signal | Direct to motor signal wire |
| D5 | BNO055 SCL | I2C (Wire) |
| D6 | Motor 2 signal | Direct to motor signal wire |
| D7 | BU03 SPI CS | To BU03 module |
| D8 | DFPlayer TX | Through 1K resistor to DFPlayer RX pin |
| D9 | DFPlayer RX | Direct to DFPlayer TX pin |
| D10 | IR phototransistor | Analog input |
| SPI bus | BU03 SCK/MOSI/MISO/IRQ/RST | See note below |

**BU03 SPI note:** The BU03 requires 6 connections: SCK, MOSI, MISO, CS, IRQ, RST. The XIAO's hardware SPI pins overlap with D7-D10 (used for DFPlayer and IR). The designer must validate whether the default SPI bus can be remapped or if software SPI is needed for either the BU03 or the MAX7219. This is the primary feasibility risk with Option B.

---

## 5. Power Architecture

```
Battery (3.7V LiPo, JST-PH 2-pin)
   |
   +---> MCU board (via JST or VIN pad)
   |        |
   |        +---> 3.3V regulated output
   |                 |
   |                 +---> BNO055 VCC
   |                 +---> DFPlayer Mini VCC
   |                 +---> BU03 VCC (Option B only)
   |                 +---> IR phototransistor bias (Robot-B)
   |
   +---> Direct battery voltage (3.7V nominal, 3.0-4.2V range)
            |
            +---> Motor 1 VCC + GND
            +---> Motor 2 VCC + GND
            +---> MAX7219 VCC + GND (Robot-A only)
```

- No additional voltage regulators needed; the MCU board's onboard 3.3V regulator supplies all logic-level peripherals
- Motors and MAX7219 tolerate the 3.0-4.2V battery voltage range
- Total current budget estimate:
  - ESP32-S3 + BLE + UWB: ~150 mA peak
  - BNO055: ~12 mA
  - DFPlayer (playing): ~30 mA (speaker current from DFPlayer internal amp)
  - Two motors: ~200-400 mA each at load
  - MAX7219 (all LEDs on): ~100 mA
  - IR LED: ~15 mA
  - **Total peak: ~900 mA** (Robot-A), ~700 mA (Robot-B)

---

## 6. Resistors Required On-Board

| Value | Quantity per unit | Location | Purpose |
|---|---|---|---|
| 1K ohm | 1 | In series between MCU UART TX and DFPlayer RX | Voltage level protection for DFPlayer serial input |
| 200 ohm | 1 (Robot-A only) | In series between MCU GPIO and IR LED anode | Current limiting for 940 nm IR LED (~15 mA at 3.3V) |
| 100 ohm | 1 (Robot-B only) | In series between MCU GPIO and visible LED anode | Current limiting for status LED (~20 mA at 3.3V) |

---

## 7. Connectors and Headers

All externally-mounted peripherals connect to the board via pin headers or solder pads. Suggested connector types are listed but the designer may substitute equivalents.

| Connector | Pin count | Unit | Purpose | Suggested type |
|---|---|---|---|---|
| Speaker | 2 | Both | DFPlayer SPK1/SPK2 to speaker | 2-pin JST-XH or bare pads |
| LED matrix | 5 | Robot-A | VCC, GND, DIN, CLK, CS to MAX7219 module | 5-pin header or Dupont |
| Status LED | 2 | Robot-B | Signal + GND to external LED (resistor on-board) | 2-pin header |
| IR emitter | 2 | Robot-A | Signal + GND to IR LED (resistor on-board) | 2-pin header |
| IR receiver | 2 | Robot-B | Signal + GND from phototransistor | 2-pin header |
| Motor 1 | 3 | Both | Signal + VCC (battery) + GND | 3-pin servo header |
| Motor 2 | 3 | Both | Signal + VCC (battery) + GND | 3-pin servo header |
| BNO055 IMU | 4 | Both | SDA, SCL, 3V3, GND | 4-pin JST or header (matching DFRobot breakout) |
| Battery | 2 | Both | Battery +/- | JST-PH 2-pin (must be accessible from outside) |
| MCU module | varies | Both | Socket or headers for MaUWB or XIAO | Female headers matching the MCU board's pins |
| BU03 module | 24 | Both (Option B) | SMD pads for BU03 | SMD-24 footprint per BU03 datasheet |

---

## 8. Mechanical and Access Requirements

- **USB-C port** on the MCU must be accessible from outside the robot chassis for programming and serial debugging. No disassembly should be needed.
- **Battery JST connector** must be accessible for charging (if using XIAO's built-in charger) or battery swapping.
- **Board footprint target:** approximately 8 x 8 cm or smaller, to fit a compact robot chassis.
- **Mounting:** The board should have 4 corner mounting holes (M2 or M2.5) for standoff mounting to the chassis.
- **Component height:** The MaUWB board (Option A) is the tallest component at ~10 mm. All other components are shorter. Maximum stack height ~15 mm including headers.
- **Antenna clearance:** The UWB antenna (on MaUWB or BU03) should not be obstructed by metal or batteries. Position the MCU/BU03 module at a board edge or top of the chassis.

---

## 9. Design Notes for the PCB Designer

1. **Universal vs. separate boards:** The Emitter and Receiver are nearly identical. A single universal board with unpopulated pads (LED matrix header vs. single LED, IR emitter vs. IR receiver) is preferred if practical. The role can be selected by which components are soldered.

2. **Ground plane:** A continuous ground plane on one layer is recommended for signal integrity (especially BLE and UWB radio performance).

3. **Motor noise:** The DC motors generate electrical noise. Decoupling capacitors (100 nF ceramic) across each motor's power terminals are recommended, placed as close to the motor headers as possible.

4. **I2C pull-ups:** The BNO055 breakout board includes pull-up resistors. No additional pull-ups are needed on the PCB. If an external OLED is added (Option B), confirm its breakout also includes pull-ups, or add 4.7K pull-ups on SDA/SCL.

5. **DFPlayer Mini orientation:** The DFPlayer Mini module has a micro-SD card slot on one side. Ensure it is oriented so the SD card is accessible for loading audio files, or use a socket that allows removal.

6. **Battery protection:** If the XIAO (Option B) is used, its built-in LiPo charging circuit handles charge management. If the MaUWB (Option A) is used, confirm whether it has LiPo charging — if not, an external TP4056 module or similar may be needed for convenient charging.

---

## 10. Bill of Materials (per unit)

### Shared (both units)

| Qty | Part | Notes |
|---|---|---|
| 1 | MCU board (MaUWB ESP32S3 or XIAO ESP32-S3) | See Section 2 |
| 1 | BU03 DW3000 module (Option B only) | SMD-24 |
| 1 | DFRobot Gravity BNO055 10DOF IMU | With JST cable |
| 2 | DFRobot Gravity DC Micro Metal Gear Motor 75:1 | Servo-style, 3-wire |
| 1 | DFPlayer Mini MP3 Player | With micro-SD card loaded with audio files |
| 1 | Small 8 ohm speaker | External mount |
| 1 | 3.7V LiPo battery, JST-PH | Robot-A: 2000 mAh. Robot-B: 1000 mAh |
| 1 | 1K ohm resistor (1/4W) | DFPlayer TX series resistor |
| 2 | 100 nF ceramic capacitor | Motor decoupling (one per motor) |
| 1 | PCB or perf board | Per this spec |

### Robot-A (Emitter) additional

| Qty | Part | Notes |
|---|---|---|
| 1 | MAX7219 8x8 LED matrix module | External mount, 5-wire cable |
| 1 | 940 nm IR LED | Forward-facing, 2-wire cable |
| 1 | 200 ohm resistor (1/4W) | IR LED current limiting |

### Robot-B (Receiver) additional

| Qty | Part | Notes |
|---|---|---|
| 1 | Visible LED (any color) | External mount, 2-wire cable |
| 1 | IR phototransistor (940 nm) | Forward-facing, 2-wire cable |
| 1 | 100 ohm resistor (1/4W) | Status LED current limiting |

---

## 11. Reference Photos

*Photos of the current breadboard prototype will be provided separately. They show:*

- [ ] Robot-A (Emitter) — top view showing component layout and wiring
- [ ] Robot-A (Emitter) — side view showing stack height
- [ ] Robot-B (Receiver) — top view
- [ ] Robot-B (Receiver) — side view
- [ ] MaUWB ESP32S3 board close-up showing IO pin labels
- [ ] Motor mounting and wiring
- [ ] BNO055 breakout board and JST cable
- [ ] DFPlayer Mini with speaker connection
- [ ] MAX7219 LED matrix module
- [ ] IR LED and phototransistor placement (forward-facing)

---

## 12. Deliverables Requested from Designer

1. **Schematic** for each unit (or one universal schematic with BOM variants marked)
2. **PCB layout** (preferred) or **perf board wiring diagram** (acceptable for prototype)
3. **Bill of Materials** with sourcing suggestions
4. **Assembly notes** for any tricky components (BU03 SMD soldering, DFPlayer orientation, etc.)
5. Confirmation of **pin assignment feasibility** for Option B (XIAO + BU03) before committing to that path
