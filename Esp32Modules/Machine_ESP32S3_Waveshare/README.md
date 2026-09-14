# Machine module – Waveshare ESP32-S3-ETH-8DI-8RO

AgOpenGPS machine module (sections, hydraulic lift, tramlines, geo stop) for the
[Waveshare ESP32-S3-ETH-8DI-8RO(-C)](https://www.waveshare.com/wiki/ESP32-S3-ETH-8DI-8RO-C)
DIN-rail board: 8 relays, 8 isolated digital inputs, W5500 Ethernet, 7–36 V supply.

It speaks the same UDP PGNs as the other AgOpenGPS machine modules, so AgIO/AgOpenGPS
see it as a normal machine module.

## Features

- 8 relays, each assignable in AgOpenGPS (Machine → Pin config): sections 1–16,
  hydraulic lower/raise, tramline right/left, geo stop. Default: relay 1–8 = section 1–8.
- Machine config (raise/lower time, relay polarity) and pin config are saved on the board.
- Relays drop to the "off" state 1 s after AgOpenGPS stops sending.
- Subnet change and module scan from AgIO work.
- Web page at `http://192.168.5.123`: status, relay and input states, relay test mode,
  subnet setting, firmware update (OTA).
- RGB LED: green = receiving AgOpenGPS data, blue = Ethernet up but no data,
  red = no Ethernet link, blinking red = relay expander or W5500 not found,
  purple = web relay test.

The digital inputs are shown on the web page but are not used for anything yet.

## Pinout

| Function | ESP32-S3 |
|---|---|
| Relays 1–8 | TCA9554PWR I²C expander at 0x20, SDA = GPIO42, SCL = GPIO41 |
| Digital inputs 1–8 | GPIO4–11, active low (input pull-up) |
| W5500 Ethernet | MISO 14, MOSI 13, SCLK 15, CS 16, INT 12 |
| WS2812 RGB LED | GPIO38 |
| Buzzer | GPIO46 |

## Building

### Arduino IDE

Needs the esp32 board package 2.0.x (tested with 2.0.14). No extra libraries.

- Board: **ESP32S3 Dev Module**
- Flash Size: **16MB**
- PSRAM: **OPI PSRAM**
- Partition Scheme: **16M Flash (3MB APP/9.9MB FATFS)**
- USB CDC On Boot: **Enabled**

The board's USB-C port is the ESP32-S3's own USB. If the upload doesn't start, hold
BOOT while pressing RESET.

### PlatformIO

```
pio run -t upload --upload-port COMx
```

## Firmware update over Ethernet

Open the web page, choose the `.bin` (Arduino IDE: Sketch → Export Compiled Binary;
PlatformIO: `.pio/build/waveshare_s3_8di8ro/firmware.bin`) and press Upload.
Relays switch off during the update.
