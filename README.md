| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-H4 | ESP32-P4 | ESP32-S2 | ESP32-S3 | Linux |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- | -------- | ----- |

# Self-Balancing Robot

ESP-IDF firmware workspace for a dual-wheel self-balancing robot on ESP32-S3.

Current focus:

- TMC2240 SPI bring-up for left/right stepper drivers
- Step/dir motor testing with timed run/stop behavior
- BNO08x I2C bring-up using the SH-2 stack
- Control-loop architecture scaffolding for balancing firmware

## How to build and flash

Use your existing ESP-IDF setup and run standard commands from the project root:

- `idf.py set-target esp32s3` (first time only)
- `idf.py build`
- `idf.py -p <PORT> flash monitor`

The active application entrypoint is [main/src/main.c](main/src/main.c). It initializes the BNO08x over I2C, initializes both TMC2240 drivers over SPI, applies the basic TMC configuration, and runs the step/dir motors for 10 seconds before disabling them.

## Motor test mode

The current bench-test firmware is focused on getting the motors spinning safely and repeatably:

- TMC SPI bus: `SPI2_HOST`
- SPI pins: `SCLK=GPIO16`, `MOSI=GPIO15`, `MISO=GPIO10`
- Left driver CS: `GPIO12`
- Right driver CS: `GPIO39`
- Shared enable: `GPIO18` (`ENN`, active-low)
- Left step/dir: `GPIO8` / `GPIO13`
- Right step/dir: `GPIO2` / `GPIO1`

At startup the firmware configures both TMC2240s, enables the motor outputs, commands a constant step rate, and then disables the motors after 10 seconds. Logging is tagged as `motor_test`.

## Repository structure

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt` files that provide set of directives and instructions describing the project's source files and targets (executable, library, or both).

Main files and folders:

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── include
│   └── src
└── README.md
```

The code under `main/src/hal/` contains the IMU transport layer, TMC2240 SPI helpers, and step/dir motor control. The main entrypoint in [main/src/main.c](main/src/main.c) drives the current motor test and still polls the IMU in the background.

For ESP-IDF project internals, see the official build system guide:
[https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)
