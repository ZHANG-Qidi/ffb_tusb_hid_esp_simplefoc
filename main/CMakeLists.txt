idf_component_register(
    SRCS "usb.cpp" "ffb.cpp" "simple_foc.cpp" "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio
    PRIV_REQUIRES esp_driver_gpio
)

target_compile_options(${COMPONENT_LIB} PRIVATE
    -fpermissive
    -Wno-missing-field-initializers
)
