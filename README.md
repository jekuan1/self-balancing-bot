| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-C61 | ESP32-H2 | ESP32-H21 | ESP32-H4 | ESP32-P4 | ESP32-S2 | ESP32-S3 | Linux |
| ----------------- | ----- | -------- | -------- | -------- | -------- | --------- | -------- | --------- | -------- | -------- | -------- | -------- | ----- |

# Self-Balancing Robot

ESP-IDF firmware workspace for a dual-wheel self-balancing robot on ESP32-S3.

Current focus:

- BNO08x I2C bring-up using the SH-2 stack
- Streaming ARVR stabilized rotation vector data as yaw/pitch/roll
- Control-loop architecture scaffolding for balancing firmware

## How to build and flash

Use your existing ESP-IDF setup and run standard commands from the project root:

- `idf.py set-target esp32s3` (first time only)
- `idf.py build`
- `idf.py -p <PORT> flash monitor`

The active application entrypoint is [main/src/main.c](main/src/main.c). It initializes the BNO08x over I2C, probes the sensor, enables the stabilized rotation-vector report, and logs yaw/pitch/roll data.

## Repository structure

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt` files that provide set of directives and instructions describing the project's source files and targets (executable, library, or both).

Main files and folders:

```
├── CMakeLists.txt
├── pytest_hello_world.py
├── main
│   ├── CMakeLists.txt
│   ├── include
│   ├── src
│   └── third_party
└── README.md
```

The legacy hello-world entrypoint has been removed. The code under `main/src/hal/` contains the IMU transport layer and SH-2 glue, while `main/src/main.c` drives the current sensor demo.

For ESP-IDF project internals, see the official build system guide:
[https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html)
