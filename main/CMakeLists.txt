idf_component_register(
    SRCS "espnow_example_main.c" "uart.cpp" "usb.cpp" "ffb.cpp" "simple_foc.cpp" "main.cpp" "interface.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_uart nvs_flash esp_event esp_netif esp_wifi
    PRIV_REQUIRES esp_driver_gpio
)
target_compile_options(${COMPONENT_LIB} PRIVATE
    -fpermissive
    -Wno-missing-field-initializers
)
