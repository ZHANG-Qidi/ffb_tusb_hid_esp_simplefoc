file(GLOB_RECURSE ARDUINO_CORE_SRC
    ${CMAKE_CURRENT_LIST_DIR}/*.c
    ${CMAKE_CURRENT_LIST_DIR}/*.cpp
)

idf_component_register(
    SRCS ${ARDUINO_CORE_SRC}
    INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/cores/arduino
    ${CMAKE_SOURCE_DIR}
    REQUIRES
    esp_driver_gpio
    esp_driver_uart
    esp_driver_i2c
    esp_driver_spi
    esp_driver_ledc
    esp_timer
)
