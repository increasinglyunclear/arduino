# Love Robots

> They are fragile. Their paths are unpredictable. They seek each other, sometimes they succeed, often they fail.

Two small autonomous robots try to seek and find each other. Made by <a href="https://philosophymachines.com/loverobots/">Philosophy Machines</a>.

The robots are not identical:

| | Robot A: **Emitter** (UWB tag T0) | Robot B: **Receiver** (UWB tag T1) |
|---|---|---|
| Body | 8x8 LED grid topped with a crystal cube | Black quartz on the front, mirror shard on the back, one green LED (Moi by Despina Papadopoulos) |
| Character | Expressive, searching, reactive | Still, watchful, minimal |
| Sketch | `mauwb_emitter` | `mauwb_receiver` |

A third device, the **Anchor**, connects the laptop to the robots. The laptop talks to it over USB serial, and it talks to the robots over Bluetooth LE.

## Emergent behavior

This applied only to the **affection** sketches. Each robot moves through three emotional states on a timer. 

* **Hesitation** (0 to 30 s): sparse, uncertain, things start and stop.
* **Ambivalence** (30 to 90 s): oscillating, neither committing nor withdrawing.
* **Excitement** (90 s onward): energetic, spreading, committed.

The robots start at slightly different times, so they are in different phases at different moments. That asynchrony is not a bug, it is the relationship.

Each state has its own movement palette per robot, so the two feel like different beings. In this code the timed sequence is composed by the **Performance** runner in the browser interface. It is stored as ordinary motion and animation sequences, which can be exported to the robots' firmware as autoplay sequences (see below), so the robots can run it without a laptop.

## System overview

```
 Browser (index.html, Chrome)
        │  Web Serial (USB)
        ▼
 Anchor (XIAO ESP32-S3 Plus + MaUWB DW3000 chipset)
        │  BLE advertising ("AnchorCmd")  ▲ BLE position broadcasts
        ▼                                  │
 Robot A (T0, Emitter)  ◄── UWB ranging ──►  Robot B (T1, Receiver)
```

* Each robot is a **Makerfabs MaUWB ESP32S3** board (ESP32-S3, STM32 and DW3000 UWB chip, controlled by AT commands), with a BNO055 IMU, two DC gear motors driven by servo-style PWM, a DFPlayer Mini for audio, and an IR sensor (A) or IR LED beacon (B).
* Robot A also has a MAX7219 8x8 LED matrix. Robot B has a status LED.
* The robots broadcast their ranging data over BLE. The anchor relays it to the browser and sends the browser's commands back to the robots.

## Arduino code

Each sketch must sit in a folder with the same name as the sketch, for example `mauwb_emitter.ino` in a folder called `mauwb_emitter`.

| Device | Sketch |
|---|---|
| Emitter (cube) robot | [`mauwb_emitter.ino`](https://github.com/increasinglyunclear/arduino/blob/main/robots/mauwb_emitter.ino) |
| Receiver (stone) robot | [`mauwb_receiver.ino`](https://github.com/increasinglyunclear/arduino/blob/main/robots/mauwb_receiver.ino) |
| Anchor (laptop ↔ BLE ↔ robots) | [`mauwb_anchor.ino`](https://github.com/increasinglyunclear/arduino/blob/main/robots/mauwb_anchor.ino) |
| Browser interface | [`index.html`](https://github.com/increasinglyunclear/arduino/blob/main/robots/index.html) |

The robot sketches also need `autoplay_sequences.h` in their folders (see *Saving sequences* below).

**Libraries** (Arduino Library Manager): Adafruit BNO055, Adafruit Unified Sensor, Adafruit BusIO, Adafruit GFX, Adafruit SSD1306, DFRobotDFPlayerMini, LedControl (Emitter only). BLE comes with the ESP32 board package.

**Boards**
* Robots: Makerfabs MaUWB ESP32S3, or an ESP32S3 Dev Module with matching settings.
* Anchor: XIAO ESP32S3 with the XIAO ESP32-S3 Plus selected. Set **Tools → USB CDC On Boot → Enabled**.

**Anchor wiring** (XIAO ESP32S3 Plus to MaUWB DW3000 chipset): 3V3 to VCC, GND to GND, D7 (GPIO44) to chipset UART1 TX (optional, for monitoring). The chipset is pre-configured as an anchor over ST-Link.

## Creating, playing and saving motion and animation sequences

1. Open Terminal on Mac.
2. Click `index.html` in the Finder, choose Get Info, and copy the path.
3. In Terminal, type `cd ` and paste the path, to go to that directory.
4. Run:
   ```bash
   python3 -m http.server 8000
   ```
5. In Chrome, go to <http://localhost:8000/>.
6. Plug in the Seeed Studio microcontroller (the anchor) with its antenna attached.
7. Click **Connect** and select the port the microcontroller is on.

## Saving sequences to run automatically on each robot

1. Create a sequence as above, name it, and press **Save**.
2. Click **Export fw**. The browser downloads `autoplay_sequences.h` to your download location.
3. Copy that file into the Emitter, Receiver and browser folders.
4. Look in `autoplay_sequences.h` for the sequence numbers. They are listed in its header comment. This list is authoritative, and the shorter list in the sketch comment may be out of date.
5. Set these lines near the top of each robot sketch, then re-upload:

```cpp
#define AUTOPLAY             1      // 1 = play a sequence automatically on power-up, 0 = don't
#define AUTOPLAY_DELAY_MS    10000  // wait this long after power-on before starting
#define AUTOPLAY_SEQUENCE    0      // sequence number from autoplay_sequences.h (0 disables autoplay)
#define AUTOPLAY_REPEAT      1      // 1 = once, 2+ = repeat N times, 0 = loop forever
```

## Adjusting motor bias (robot does not travel straight)

1. In the browser interface, drive the robot forward manually.
2. Adjust the **Bias** slider for that robot until it goes straight.
3. Note the number and put it in the robot's sketch:

```cpp
int8_t motorBiasPct = 0;   // replace 0 with the number from the slider
```

## Troubleshooting

**Upload fails with "Failed to connect to ESP32-S3: No serial data received."** The port shows up but the chip isn't answering, usually because the sketch already on the board is stopping auto-reset. Put the board in bootloader mode by hand: unplug it, hold **B** (BOOT), plug it in while holding B, release after a second, reselect the port and upload. Alternatively hold **B**, tap **R** (RESET), then release B. Afterwards, press **R** or re-plug to start the new sketch. If this keeps happening, try **Erase All Flash Before Sketch Upload**, a different cable, and no USB hub.

**Anchor LED.** The XIAO's user LED (GPIO21) blinks every 2 seconds while the sketch is running. The red LED beside the USB-C port is the battery charge indicator and is not controlled by the code.

**Check the anchor is alive.** Open the Serial Monitor at 115200 baud. You should see `=== MaUWB Anchor + BLE Relay + Cmd ===` and then a `SCAN:` status line every 3 seconds.

## Experimental: emergent behavior and learning (the `affection` set)

These sketches add an **Emergent mode** to the robots, in which each robot's behavior is coupled to what it observes the other doing. A small synchrony learner adjusts three probabilities (mirror, adopt, drift) over time, and Emergent mode is stored on the robot so it survives a reboot. The files are `mauwb_emitter_affection`, `mauwb_receiver_affection` and `mauwb_anchor_affection`, plus a matching `position_map/index.html` with Emergent on/off controls.

> **Status: not yet rigorously tested.** Use the basic set above for anything that matters.

## IR experiments

Tests of 38 kHz infrared signalling, for seeking by line of sight:

* `Uno_IR_emitter`: an Arduino Uno drives a 940 nm IR LED through a BC547 transistor with 38 kHz bursts, like a TV remote.
* `Nano_IR_receiver`: an Arduino Nano 33 BLE reads a TSOP38238 or TSOP4838 demodulator and prints detect or idle on Serial (115200 baud). The two boards need no shared ground wire.

The wiring for each is in the header comment of its sketch.

## Credits

Made with love, by Philosophy Machines.
