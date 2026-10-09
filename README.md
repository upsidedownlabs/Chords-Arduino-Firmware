# Chords Arduino Firmware

**Chords** is an open-source toolkit developed by Upside Down Labs to transform Arduino-compatible 
boards into bio-potential data acquisition devices when paired with BioAmp hardware.

## Tools

1. [Chords-Web](https://chords.upsidedownlabs.tech/)
2. [Chords-Python](https://github.com/upsidedownlabs/Chords-Python)

> [!NOTE]
> You have to flash Arduino code to your hardware from the list below to use these tools.
> [![](https://img.youtube.com/vi/INTXVJh3pEQ/maxresdefault.jpg)](https://youtu.be/INTXVJh3pEQ?si=0HVZ6AT-9xdLbCwc)

## Supported boards

> [!IMPORTANT]
> Make sure to select your board type in the firmware file for it to work properly.

> [!TIP]
> Only use genuine board to avoid noisy (unusable) signals and connection issues.

| Board | Voltage | Channels | Resolution | SamplingRate | BaudRate | Code |
| ----- | ------- | -------- | ---------- | ------------ | -------- | ---- |
| Neuro Play Ground (NPG) Lite - Serial | 2V5 | 3-6 | 12-bit | 500 | 230400 | [NPG-LITE-Serial.ino](NPG-LITE-Serial/NPG-LITE-Serial.ino) |
| Neuro Play Ground (NPG) Lite - BLE | 2V5 | 3-6 | 12-bit | 500 | - | [NPG-LITE-BLE.ino](NPG-LITE-BLE/NPG-LITE-BLE.ino) |
| Neuro Play Ground (NPG) Lite - WiFi | 2V5 | 3-6 | 12-bit | 500 | - | [NPG-LITE-WiFi.ino](NPG-LITE-WiFi/NPG-LITE-WiFi.ino) |
| STM32G4 Core Board | 3V3 | 16 | 12-bit | 500 | 230400 | [STM32G4-CORE-BOARD.ino](STM32G4-CORE-BOARD/STM32G4-CORE-BOARD.ino) |
| STM32F4 Black Pill | 3V3 | 8 | 12-bit | 500 | 230400 | [STM32F4-BLACK-PILL.ino](STM32F4-BLACK-PILL/STM32F4-BLACK-PILL.ino) |
| Arduino GIGA R1 (WiFi) | 3V3 | 6 | 16-bit | 500 | 230400 | [GIGA-R1.ino](GIGA-R1/GIGA-R1.ino) |
| Raspberry PI Pico | 3V3 | 3 | 12-bit | 500 | 230400 | [RPI-PICO-RP2040.ino](RPI-PICO-RP2040/RPI-PICO-RP2040.ino) |
| Arduino UNO R4 Minima/WiFi | 5V | 6 | 14-bit | 500 | 230400 | [UNO-R4.ino](UNO-R4/UNO-R4.ino) |
| Arduino NANO Classic | 5V | 8 | 10-bit | 250 | 115200 | [AVR-NANO-UNO-MEGA.ino](AVR-NANO-UNO-MEGA/AVR-NANO-UNO-MEGA.ino) |
| Arduino UNO R3 | 5V | 6 | 10-bit | 250 | 115200 | [AVR-NANO-UNO-MEGA.ino](AVR-NANO-UNO-MEGA/AVR-NANO-UNO-MEGA.ino) |
| Arduino Genuino UNO | 5V | 6 | 10-bit | 250 | 115200 | [AVR-NANO-UNO-MEGA.ino](AVR-NANO-UNO-MEGA/AVR-NANO-UNO-MEGA.ino) |
| Arduino MEGA 2560 R3 | 5V | 16 | 10-bit | 250 | 115200 | [AVR-NANO-UNO-MEGA.ino](AVR-NANO-UNO-MEGA/AVR-NANO-UNO-MEGA.ino) |
| Maker Nano / Nano Clone (CH340) | 5V | 8 |  10-bit | 250 | 115200 | [AVR-NANO-UNO-MEGA.ino](AVR-NANO-UNO-MEGA/AVR-NANO-UNO-MEGA.ino) |
| Maker UNO / UNO R3 Clone (CH340) | 5V | 6 | 10-bit | 250 | 115200 | [AVR-NANO-UNO-MEGA.ino](AVR-NANO-UNO-MEGA/AVR-NANO-UNO-MEGA.ino) |
| MEGA 2560 Clone (CH340) | 5V | 16 | 10-bit | 250 | 115200 | [AVR-NANO-UNO-MEGA.ino](AVR-NANO-UNO-MEGA/AVR-NANO-UNO-MEGA.ino) |
| ESP32-S3 | 3V3 | 16 | 12-bit | 500 | 230400 | [ESP32-S3.ino](ESP32-S3/ESP32-S3.ino) |

## Prebuilt firmware

Prebuilt binaries are published on the [Releases](../../releases) page. Download the file matching your board:

| Board | File | How to flash |
| ----- | ---- | ------------ |
| NPG Lite Serial (ESP32-C6) | [Chords-NPG-LITE-Serial-ESP32C6.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-NPG-LITE-Serial-ESP32C6.bin) | esptool / [NPG-Lite-Flasher-Web](https://upsidedownlabs.github.io/NPG-Lite-Flasher-Web/), offset `0x0` |
| NPG Lite BLE (ESP32-C6) | [Chords-NPG-LITE-BLE-ESP32C6.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-NPG-LITE-BLE-ESP32C6.bin) | esptool / [NPG-Lite-Flasher-Web](https://upsidedownlabs.github.io/NPG-Lite-Flasher-Web/), offset `0x0` |
| NPG Lite WiFi (ESP32-C6) | [Chords-NPG-LITE-WiFi-ESP32C6.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-NPG-LITE-WiFi-ESP32C6.bin) | esptool / [NPG-Lite-Flasher-Web](https://upsidedownlabs.github.io/NPG-Lite-Flasher-Web/), offset `0x0` |
| ESP32-S3 | [Chords-ESP32-S3.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-ESP32-S3.bin) | esptool / [NPG-Lite-Flasher-Web](https://upsidedownlabs.github.io/NPG-Lite-Flasher-Web/), offset `0x0` |
| STM32G4 Core Board (G431CB) | [Chords-STM32G4-CORE-BOARD.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-STM32G4-CORE-BOARD.bin) | STM32CubeProgrammer / DFU, `0x08000000` |
| STM32F4 Black Pill (F401CC / F411CE) | [Chords-STM32F401CC-BLACK-PILL.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-STM32F401CC-BLACK-PILL.bin) / [Chords-STM32F411CE-BLACK-PILL.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-STM32F411CE-BLACK-PILL.bin) | STM32CubeProgrammer / DFU, `0x08000000` |
| Arduino GIGA R1 | [Chords-GIGA-R1.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-GIGA-R1.bin) | `dfu-util` |
| Raspberry Pi Pico | [Chords-RPI-PICO-RP2040.uf2](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-RPI-PICO-RP2040.uf2) | Hold BOOTSEL, drag & drop the `.uf2` |
| Arduino UNO R4 Minima / WiFi | [Chords-UNO-R4-MINIMA.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-UNO-R4-MINIMA.bin) / [Chords-UNO-R4-WIFI.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-UNO-R4-WIFI.bin) | `bossac` / Arduino IDE |
| Arduino UNO R3 | [Chords-UNO-R3.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-UNO-R3.bin) | `avrdude -U flash:w:<file>:r` |
| Arduino Nano Classic | [Chords-NANO-CLASSIC.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-NANO-CLASSIC.bin) | `avrdude -U flash:w:<file>:r` |
| Arduino MEGA 2560 R3 | [Chords-MEGA-2560-R3.bin](https://github.com/upsidedownlabs/Chords-Arduino-Firmware/releases/latest/download/Chords-MEGA-2560-R3.bin) | `avrdude -U flash:w:<file>:r` |
