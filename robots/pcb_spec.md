# Robot PCB / Perf Board Specification
Kevin Walker 30 Mar 2026

---

## 1. Project Overview

Two small autonomous robots that find and navigate toward each other using UWB (Ultra-Wideband) distance ranging, BLE (Bluetooth Low Energy), and an IMU for orientation. They need custom PCBs or perf boards to replace the current breadboard/flying-wire prototypes.

- **Robot-A (Emitter / UWB Tag)** — the seeker. Has an 8x8 LED matrix for directional arrows, an IR emitter LED, motors, IMU, and audio.
- **Robot-B (Receiver / UWB Anchor)** — the target. Has a single status LED, an IR phototransistor receiver, motors, IMU, and audio.

Both units are battery-powered (3.7V LiPo) and must be programmable via USB-C without disassembly.

---

## 2. MCU: Makerfabs MaUWB ESP32S3 

- https://www.makerfabs.com/mauwb-esp32s3-uwb-module.html
- **What it is:** Single board integrating ESP32-S3 + STM32 coprocessor + DW3000 UWB chip + 0.96" OLED (128x64)
- **Footprint:** ~60 x 25 mm (with headers)
- **UWB interface:** AT commands over internal UART (GPIO17/18), transparent to the designer
- **Available GPIOs for peripherals:** IO1 through IO11
- **Power input:** USB-C on-board, or VIN/GND pins

---

## 3. Peripheral List

### 3.1 Shared Peripherals (both Robot-A and Robot-B)

| # | Peripheral | Model / Part | Interface | Power | Notes |
|---|---|---|---|---|---|
| 1 | IMU | DFRobot Gravity 10DOF (BNO055) | I2C, address 0x28 | 3.3V from MCU | Breakout board with JST cable. SDA/SCL/VCC/GND| https://grobotronics.com/gravity-10-dof-imu-ahrs-bno055-bmp280.html
| 2 | Motor 1 | DFRobot Gravity DC Micro Metal Gear Motor 75:1 | Servo-style PWM, 1000-2000 us | VCC/GND direct from battery | Signal wire only from MCU | https://grobotronics.com/gravity-dc-micro-metal-gear-motor-w-driver-75-1.html
| 3 | Motor 2 | DFRobot Gravity DC Micro Metal Gear Motor 75:1 | Servo-style PWM, 1000-2000 us | VCC/GND direct from battery | Signal wire only from MCU | https://grobotronics.com/gravity-dc-micro-metal-gear-motor-w-driver-75-1.html
| 4 | Audio player | DFPlayer Mini MP3 | UART 9600 baud | 3.3V from MCU | TX line needs 1K ohm series resistor. RX direct. | https://wiki.dfrobot.com/dfr0299
| 5 | Speaker | Small 8 ohm speaker | Wired to DFPlayer SPK1/SPK2 | From DFPlayer | **Mounted externally**, connected via 2-wire cable | https://grobotronics.com/speaker-enclosed-2w-8ohm-20x30mm-ph1-25.html
| 6 | Battery | 3.7V LiPo, JST-PH 2-pin | JST connector | N/A | Robot-A: 2000 mAh. Robot-B: 1000 mAh. | https://grobotronics.com/li-po-battery-3-7v-2000mah-molex-2-5mm.html

### 3.2 Robot-A (Emitter) Only

| # | Peripheral | Model / Part | Interface | Power | Notes |
|---|---|---|---|---|---|
| 7 | LED matrix | MAX7219 8x8 LED module | Bit-banged SPI (DIN, CLK, CS) | VCC/GND direct from battery | **Mounted externally**, connected via 5-wire cable | https://grobotronics.com/led-matrix-8x8-red-with-max7219.html
| 8 | IR emitter | 940 nm IR LED | GPIO + 200 ohm series resistor | From GPIO | Mounted forward-facing on chassis, short wire to board | https://www.hellasdigital.gr/electronics/sensors/infrared-sensors/940nm-ir-infrared-receiver-led/

### 3.3 Robot-B (Receiver) Only

| # | Peripheral | Model / Part | Interface | Power | Notes |
|---|---|---|---|---|---|
| 7 | Status LED | Standard visible LED (any color) | GPIO + 100 ohm series resistor | From GPIO | **Mounted externally**, 2-wire cable | (to be provided)
| 8 | IR receiver | IR phototransistor (940 nm sensitive) | Analog input | 3.3V bias | Mounted forward-facing on chassis, short wire to board | https://www.hellasdigital.gr/electronics/sensors/infrared-sensors/940nm-ir-infrared-receiver-led/
---

## 4. Pin Mapping

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
   |                 +---> IR phototransistor bias (Robot-B)
   |
   +---> Direct battery voltage (3.7V nominal, 3.0-4.2V range)
            |
            +---> Motor 1 VCC + GND
            +---> Motor 2 VCC + GND
            +---> MAX7219 VCC + GND (Robot-A only)
            +---> DFPlayer Mini VCC
```

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

---

## 8. Mechanical and Access Requirements

- **USB-C port** on the MCU must be accessible from outside the robot chassis for programming and serial debugging. No disassembly should be needed.
- **Battery JST connector** must be accessible for charging or battery swapping.
- **Board footprint target:** approximately 8 x 8 cm or smaller, to fit a compact robot chassis.
- **Component height:** The MaUWB board is the largest component at ~10 mm length. All other components are shorter. Stacking is possible: BNO055 is wired via JST connector not pins, same with LED matrix (Robot-A) which needs to be on top - it can be above the MaUWB leaving the OLED visible.
- **Antenna clearance:** The UWB antenna (on MaUWB) should not be obstructed by metal or batteries. Position the MCU/BU03 module at a board edge.

---

## 9. Design Notes for the PCB Designer

1. **Universal vs. separate boards:** The Emitter and Receiver are nearly identical. A single universal board with unpopulated pads (LED matrix header vs. single LED, IR emitter vs. IR receiver) is preferred if practical. The role can be selected by which components are soldered.

2. **Ground plane:** A continuous ground plane on one layer is recommended for signal integrity (especially BLE and UWB radio performance).

3. **Motor noise:** The DC motors generate electrical noise. Decoupling capacitors (100 nF ceramic) across each motor's power terminals could be used, placed as close to the motor headers as possible.

4. **I2C pull-ups:** The BNO055 breakout board includes pull-up resistors. No additional pull-ups are needed on the PCB. 

5. **DFPlayer Mini orientation:** The DFPlayer Mini module has a micro-SD card slot on one side. Ensure it is oriented so the SD card is accessible for loading audio files, or use a socket that allows removal.

---

## 10. Bill of Materials (per unit)

### Shared (both units)

| Qty | Part | Notes |
|---|---|---|
| 1 | MCU board (MaUWB ESP32S3) |
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
