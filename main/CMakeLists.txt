idf_component_register(
    SRCS
    "wheel_registry.c"
    "espnow_example_main.c"
    "uart.cpp"
    "usb.cpp"
    "ffb.cpp"
    "simple_foc.cpp"
    "main.cpp"
    "interface.cpp"
    "debug.cpp"
    PRIV_REQUIRES
    nvs_flash
    esp_event
    esp_netif
    esp_wifi
    esp_driver_gpio
    esp_driver_uart
    esp_timer
    esp_driver_gptimer
    REQUIRES
    ArduinoCore-avr-1.8.7
    Arduino-FOC-2.2
    INCLUDE_DIRS "."
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -fpermissive
    -Wno-missing-field-initializers
)
