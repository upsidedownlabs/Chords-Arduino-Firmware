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
| Neuro Play Ground (NPG) Lite | 2V5 | 3-6 | 12-bit | 500 | 230400 | [NPG-LITE.ino](NPG-LITE/NPG-LITE.ino) |
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

