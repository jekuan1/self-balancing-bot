idf_component_register(
    SRCS
        "src/main.c"
        "src/hal/imu_module.c"
        "src/third_party/bno08x/sh2.c"
        "src/third_party/bno08x/sh2_SensorValue.c"
        "src/third_party/bno08x/sh2_util.c"
        "src/third_party/bno08x/shtp.c"
    INCLUDE_DIRS
        "include"
        "src/third_party/bno08x"
    REQUIRES driver esp_driver_gpio esp_driver_spi esp_timer
)
