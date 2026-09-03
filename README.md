# Cypher CC1101 Jammer — PCB & Firmware

**Open-source sub-GHz RF pentesting device** built around a **WEMOS D1 Mini (ESP8266)** and a **CC1101** radio module. A YardStick-One-like serial CLI for scanning, transmitting, jamming, brute-forcing, and RAW record/replay of sub-GHz signals — plus a browser control panel. Works alongside a Flipper Zero.

> 🛠️ Flash [`cc1101-tool-esp8266/cc1101-tool-esp8266.ino`](cc1101-tool-esp8266/cc1101-tool-esp8266.ino) if you're using the WEMOS D1 Mini.
>
> 🔗 Order my PCB from PCBWay & get a $10 coupon ^_^ : https://pcbway.com/g/87Pi52

<table>
  <tr>
    <td><img src="device_img/cypher_c11011.JPG" alt="Cypher CC1101 device" width="260"></td>
    <td><img src="device_img/cypher_c11012.JPG" alt="Cypher CC1101 device" width="260"></td>
    <td><img src="device_img/cypher_c11013.JPG" alt="Cypher CC1101 device" width="260"></td>
  </tr>
  <tr>
    <td><img src="device_img/cypher_c11014.JPG" alt="Cypher CC1101 device" width="260"></td>
    <td><img src="device_img/cypher_c11015.JPG" alt="Cypher CC1101 device" width="260"></td>
    <td valign="middle"><img src="Esp8266_CC1101.png" alt="ESP8266 + CC1101 wiring" width="260"></td>
  </tr>
</table>

---

## 🆕 What's new (2026)

A major refresh of the WEMOS D1 Mini firmware:

- **🌐 WiFi Access Point + web control panel** — the board hosts its own AP and serves a browser UI with radio config, action buttons, a buffer viewer, and a raw command console. The USB serial CLI keeps working at the same time.
- **⚡ Responsive, non-blocking modes** — `scan`, `sniff` (rxraw), `brute`, `recraw` and `playraw` now run as background modes, so the web UI no longer freezes and `recraw` no longer hangs waiting for a signal.
- **🔧 Builds on modern toolchains again** — the sketch compiles cleanly on the current Arduino ESP8266 core / `arduino-cli`, with a pinned `sketch.yaml` build profile.
- **🐞 Bug fixes** — out-of-bounds writes, chat/brute overflows, `load` frame-count restore, and the long-standing *"brute hangs after the full cycle"* GDO0 pin-contention bug (single-cycle brute is reliable now). Same build + safety fixes applied to every board variant in `original_files/`.

Full details in the [changelog](#-changelog) at the bottom.

---

## 🔌 USB serial mode

Plug the board into **USB** and drive it with a serial terminal at **115200 baud** — no WiFi needed. This works two ways:

- **Computer** — PuTTY, `screen`, the Arduino IDE Serial Monitor, or any terminal. Ports: `/dev/ttyACM0` / `/dev/cu.*` on Linux/macOS, `COMxx` on Windows.
- **Android phone (USB-OTG)** — plug the board into your phone and use a [USB serial terminal app](https://play.google.com/store/apps/details?id=de.kai_morich.serial_usb_terminal) for portable hacking.

**Important:** set the terminal newline to **CR only** — extra LF characters can break commands like `rxraw` (in the Android app: Settings → Newline → CR). Send `help` for the command list, and run `init` after any RAW-mode operation.

---

## 🌐 Web UI (AP mode)

Flash the D1 Mini sketch and the board creates its own WiFi Access Point:

| | |
|---|---|
| **SSID** | `cc1101` |
| **Password** | `cc1101` |
| **URL** | http://192.168.1.100 |

Join that network from your phone or laptop and open the URL for a control panel — radio config, action buttons, buffer view, and a raw command console (everything reuses the same CLI). The USB serial console works at the same time. Tip: run the ESP8266 at **160 MHz** for best WiFi stability.

---

## ⚡ Build & flash

**With `arduino-cli` (reproducible, recommended):** the pinned [`sketch.yaml`](cc1101-tool-esp8266/sketch.yaml) profile installs the ESP8266 core and the SmartRC library automatically.

```bash
arduino-cli compile --profile d1mini cc1101-tool-esp8266
arduino-cli upload -p <PORT> --profile d1mini cc1101-tool-esp8266
```

**With the Arduino IDE:** install the **SmartRC-CC1101-Driver-Lib** (ELECHOUSE fork by Little_S@tan — https://github.com/LSatan/SmartRC-CC1101-Driver-Lib), add ESP8266 board support, select the WEMOS D1 Mini, set CPU frequency to **160 MHz**, then compile & upload. A "Low Memory" warning is expected and harmless.

Once flashed, connect over [USB serial](#-usb-serial-mode) or the [Web UI](#-web-ui-ap-mode) above.

---

## 📟 Command reference

<details>
<summary><b>Radio configuration</b></summary>

| Command | Description |
|---------|-------------|
| `setmodulation <mode>` | 0 = 2-FSK, 1 = GFSK, 2 = ASK/OOK, 3 = 4-FSK, 4 = MSK |
| `setmhz <frequency>` | Base frequency (default 433.92). CC1101 bands: 300–348, 387–464, 779–928 MHz |
| `setdeviation <kHz>` | Frequency deviation, 1.58–380.85 kHz (default 47.60) |
| `setchannel <0-255>` | Channel number (default 0) |
| `setchsp <spacing>` | Channel spacing in kHz, 25.39–405.45 (default 199.95) |
| `setrxbw <kHz>` | Receive bandwidth, 58.03–812.50 kHz (default 812.50) |
| `setdrate <kBaud>` | Data rate, 0.02–1621.83 kBaud |
| `setpa <dBm>` | TX power: -30 -20 -15 -10 -6 0 5 7 10 11 12 (default max) |
| `applyradio <MHz> <mod> <rate> <dev> <bw> <dBm>` | Atomically validate, apply, calibrate, read back, and report a complete radio profile |
| `setsyncmode <0-7>` | Sync-word qualifier mode (0 = none … 7 = 30/32 + carrier-sense) |
| `setsyncword <LOW HIGH>` | Sync word (must match transmitter & receiver) |
| `setadrchk <0-3>` | Address-check config for received packets |
| `setaddr <address>` | Address used for packet filtration (broadcast: 0x00 / 0xFF) |
| `setwhitedata <0/1>` | Data whitening off / on |
| `setpktformat <0-3>` | 0 = FIFO, 1 = sync serial, 2 = random TX (PN9), 3 = async serial |
| `setlengthconfig <0-3>` | 0 = fixed, 1 = variable, 2 = infinite length mode |
| `setpacketlength <n>` | Packet length (fixed mode) / max length (variable mode) |
| `setcrc <0/1>` | CRC calculation/check off / on |
| `setcrcaf <0/1>` | Auto-flush RX FIFO on bad CRC |
| `setdcfilteroff <0/1>` | Digital DC blocking filter (only for data rates ≤ 250 kBaud) |
| `setmanchester <0/1>` | Manchester encoding/decoding off / on |
| `setfec <0/1>` | Forward Error Correction off / on (fixed length only) |
| `setpre <0-7>` | Minimum preamble bytes (0:2 … 7:24) |
| `setpqt <mode>` | Preamble quality estimator threshold |
| `setappendstatus <0/1>` | Append RSSI/LQI/CRC status bytes to payload |
| `getrssi` | Show radio quality info for the last received frame |

The web Apply button uses `applyradio`, so all profile registers are written only after the CC1101 reaches IDLE. RX bandwidth is snapped to the nearest legal CC1101 value, frequency changes receive an explicit synthesizer calibration, and the status panel displays register-derived frequency and bandwidth rather than the submitted form text.

</details>

<details>
<summary><b>Actions</b></summary>

| Command | Description |
|---------|-------------|
| `scan <start> <end>` | Scan a frequency range for the strongest signal (background mode) |
| `rx` | Enable/disable printing of received RF packets |
| `tx <hex-vals>` | Send a packet (max 60 bytes) of hex values over RF |
| `jam` | Enable/disable continuous jamming on the selected band |
| `brute <usec> <bits>` | Brute-force a DIP-switch gate: `<bits>` code, `<usec>` symbol length (background mode) |
| `chat` | Switch the device into IRC-like chat mode (disconnect to quit) |
| `x` | Stop jamming / receiving / recording / any background mode |
| `init` | Restart the CC1101 with default parameters |

</details>

<details>
<summary><b>Frame record / replay</b></summary>

| Command | Description |
|---------|-------------|
| `rec` | Enable/disable recording received frames into the buffer |
| `show` | Show the contents of the recording buffer |
| `add <hex-vals>` | Manually add a single frame (max 60 hex values) to the buffer |
| `flush` | Clear the recording buffer |
| `play <N>` | Replay frame N (or 0 for all recorded frames) |
| `save` | Store the recording buffer in non-volatile memory |
| `load` | Load the recording buffer back from non-volatile memory |

</details>

<details>
<summary><b>RAW record / replay (Flipper-style, URH-compatible)</b></summary>

| Command | Description |
|---------|-------------|
| `rxraw <usec>` | Sniff the radio at `<usec>` **per-sample** interval and print hex (background mode; max 1000 µs) |
| `recraw <usec>` | Record RAW RF data at `<usec>` **per-sample** interval, starts on signal (max 150 µs → ~5 s window) |
| `playraw <usec>` | Replay recorded RAW RF data at `<usec>` **per-sample** interval (max 150 µs) |
| `addraw <hex-vals>` | Manually add RAW chunks (max 60 hex values) to the buffer |
| `showraw` | Show the recording buffer in RAW hex format |
| `showbit` | Show the recording buffer as a stream of bits |
| `echo <0/1>` | Enable/disable command echo on the serial terminal |

RAW bit order matches [Universal Radio Hacker](https://github.com/jopohl/urh). The `<usec>` argument is the delay **per sample** (per bit), not a total window — a 4096-byte `recraw` spans about `usec × 32768` µs. It is required and is clamped to a watchdog-safe maximum (150 µs for `recraw`/`playraw`, 1000 µs for `rxraw`); pass an oversized value and the firmware clamps it and tells you. Run `init` after any RAW operation.

</details>

The firmware uses the **SmartRC** library (modified ELECHOUSE library by Little_S@tan) so every transmission parameter is settable in human-readable form without SmartRF Studio: https://github.com/LSatan/SmartRC-CC1101-Driver-Lib

---

## 🔌 Wiring & porting

The CC1101 requires **3.3 V VCC and 3.3 V TTL logic**. 5 V boards (e.g. Arduino Nano) need a **TXS0108E** level shifter or you will fry the CC1101.

<details>
<summary><b>Arduino Pro Micro (3.3V / 8MHz) reference wiring</b></summary>

| Pro Micro pin | CC1101 |
|---|---|
| D3 (PD0 / INT0) | GDO0 |
| D9 (PB5) | GDO2 |
| D10 (PB6) | CSN / SS |
| D16 (PB2 / MOSI) | MOSI / SI |
| D14 (PB3 / MISO) | MISO / SO |
| D15 (PB1 / SCK) | SCLK / CLK |
| VCC 3.3V | VCC |
| GND | GND |

The Pro Micro must support **3.3 V VCC and 3.3 V TTL logic** ([SparkFun setup guide](https://learn.sparkfun.com/tutorials/pro-micro--fio-v3-hookup-guide/all)). Upload trouble? Right after pressing **Upload**, short GND+RST twice within a few seconds to trigger the bootloader.

</details>

<details>
<summary><b>Pin maps & buffer sizes for other boards</b></summary>

To use a different board, change the pin assignment and buffer/EEPROM sizes at the top of the sketch.

| Signal | ESP32 | XIAO ESP32-C3 | WEMOS D1 Mini | Arduino Nano | RP2040 / Pico |
|--------|:-----:|:-------------:|:-------------:|:------------:|:-------------:|
| `sck`  | 18 | 8  | 14 | 16 (D13) | 2 |
| `miso` | 19 | 4  | 12 | 15 (D12) | 4 |
| `mosi` | 23 | 10 | 13 | 14 (D11) | 3 |
| `ss`   | 5  | 20 | 15 | 13 (D10) | 5 |
| `gdo0` | 2  | 21 | 5  | 9 (D6)   | 7 |
| `gdo2` | 4  | 7  | 4  | 5 (D2)   | 6 |
| `RECORDINGBUFFERSIZE` | 4096 | 4096 | 4096 | 1024 | 4096 |
| `EPROMSIZE` | 512 | 512 | 4096 | 1024 | 512 |

- **XIAO ESP32-C3:** cheap green D-SUN CC1101 boards may need cable shielding on GDO0, otherwise `rxraw`/`recraw` catch noise like `FF`.
- **Arduino Nano:** tested and **requires** a TXS0108E 5V↔3.3V level converter (especially for the E07-M1101D CC1101).
- **RP2040 / Pico:** tested, uses 3.3 V logic ([pinout](https://cdn-learn.adafruit.com/assets/assets/000/099/339/original/raspberry_pi_Pico-R3-Pinout-narrow.png)).

Older separate WiFi variants (telnet client / telnet AP) also live in [`original_files/`](original_files/).

</details>

---

## 📺 Videos

- First version of the project: https://youtu.be/iPVckkTjsd0
- Using Universal Radio Hacker with the CC1101-tool: https://youtu.be/mdkEK_wmWJA

---

## 📜 Changelog

<details>
<summary><b>2023 – 2024 history (click to expand)</b></summary>

**08.06.2023 — optimized CLI**
- removed unnecessary parameters for RX, TX, JAM; renamed JAMM → JAM
- RX now prints hex values directly with no description when the sniffer is enabled
- corrected CR/LF handling for the Android "Serial Terminal" app on USB OTG
- added CHAT mode (IRC-like communicator between multiple devices)

**09.06.2023 — added RAW mode (Flipper-style)**
- `rxraw`, `recraw`, `playraw`, `showraw` for record & replay attacks
- recording buffer: 1536 bytes on ATMEGA32U4, 1024 on Mega/Uno/Nano, 4096+ on ESP32
- always run `init` after RAW mode to restart the CC1101

**10.06.2023**
- added Arduino Mega/Nano/Uno version (requires TXS0108E level converter)
- added ESP32 version
- `recraw` now starts recording once something appears over the radio
- added `addraw` to compose a signal in the buffer (e.g. hex chunks from URH)
- added `scan <start> <end>` to find a peak frequency

**17.06.2023**
- added `save`/`load` for the recorded-frames buffer to/from EEPROM
- added `showbit` (RAW data as a bit stream)
- fixed ESP32 `(char *)` vs `(byte *)` type issue

**18.06.2023**
- updated bit storage order in `playraw`/`rxraw`/`recraw` to match [URH](https://github.com/jopohl/urh)

**30.06.2023**
- added CC1101 startup debug message
- corrected EEPROM usage (ESP32 512 bytes, ESP8266 4096 bytes)
- added ESP32-WROOM and ESP8266 WEMOS D1 Mini versions

**08.07.2023**
- ESP8266 WDT watchdog restarts mostly solved (single-core chip, heavy WiFi/TCP-IP load); tested on WEMOS D1 Mini clone + D-SUN CC1101

**13.07.2023**
- default packet-mode data rate 1.2 kBaud → 9.6 kBaud (fewer ESP8266 watchdog restarts)

**27.07.2023** — added RP2040 board

**18.08.2023** — added `brute <microseconds> <bits>` for DIP-switch gates. *(Sometimes the code hangs after a full brute cycle — root cause found and fixed in 2026, see below.)*

**02.09.2023** — added ESP8266 WiFi **client** mode (separate source version) — telnet to the board via an intermediary access point to extend range. Set CPU to 160 MHz.

**08.09.2023** — added ESP8266 WiFi **Access Point** mode (separate source version) — telnet to `192.168.1.100:23`, default SSID `cc1101`. Set CPU to 160 MHz.

**22.11.2023** — fixed `showbit()` bug (thanks jps1x2).

**08.02.2024** — fixed `scan()` not accepting MHz fractions in the range (thanks chris4soft).

</details>

**31.07.2026 — major refresh of the WEMOS D1 Mini (ESP8266) firmware**
- **Builds again on current toolchains** (Arduino ESP8266 core 3.x / `arduino-cli` 1.x). Old CRLF line endings + non-ASCII characters were breaking the Arduino auto-prototype generator; the source is now ASCII/LF with explicit prototypes, plus a pinned `sketch.yaml` profile.
- **Memory-safety & correctness fixes** — out-of-bounds writes in the hex conversion and the REC buffer flush, chat/brute overflows, an uninitialized SCAN comparison, and `load` not rebuilding the frame count (SHOW/PLAY work after LOAD again). Command handling de-duplicated. Same build + OOB fixes applied to every variant in `original_files/`.
- **WiFi AP + web control panel** — see [Web UI](#-web-ui-ap-mode) above.
- **Non-blocking long-running commands** — SCAN, RXRAW (sniffer), BRUTE, RECRAW and PLAYRAW run as background modes so the web UI stays responsive (and on serial they return to the prompt immediately, stopping on any key or `x`). RECRAW no longer hangs waiting for a signal.
- **Fixed the "BRUTE hangs after the full cycle" bug** — the RAW TX modes left GDO0 driven as an OUTPUT against the CC1101 (pin contention → watchdog reset). GDO0 is now released to INPUT on raw-mode exit; single-cycle brute is reliable.
- **Fixed the `recraw`/`playraw` watchdog reset** — the capture/replay `<usec>` is the delay *per sample*, so an oversized value (e.g. `recraw 4000000`) made a single `delayMicroseconds()` busy-wait for seconds and trip the software watchdog before it was ever fed. All RAW intervals are now clamped to a watchdog-safe maximum, and the continuous capture/replay block runs with the soft WDT suspended for its (now bounded) duration — so `recraw`/`playraw` complete instead of resetting the board.

---

## ⚠️ Known issues

- In packet mode, `rx` can misbehave after many large frames have been received — suspected memory leak in the SmartRC library.
- The RAW `<usec>` values are a delay *per sample*, not a total window; very large values are clamped to a watchdog-safe maximum (150 µs for `recraw`/`playraw`, 1000 µs for `rxraw`). Earlier firmware reset the board on oversized values — fixed in 2026 (see above).
- On the ESP8266 WiFi/AP build, `brute` with many bits (roughly 5+, i.e. more than 16 codes) can still trigger a watchdog reset while the AP is running — a single-core WiFi vs. bit-bang timing limitation. Small bit-counts are reliable.

---

## ⚖️ Legal & safety

This is a **security-research and education** tool. RF jamming and transmitting on unauthorized frequencies is illegal in many jurisdictions — only transmit on bands, power levels, and devices you are authorized to use. See [BEGINNER.md](BEGINNER.md) for a fuller intro and safety notes.
