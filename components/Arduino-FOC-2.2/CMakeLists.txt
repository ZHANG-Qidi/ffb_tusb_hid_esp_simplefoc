file(GLOB_RECURSE ARDUINO_FOC_SRC
    ${CMAKE_CURRENT_LIST_DIR}/*.c
    ${CMAKE_CURRENT_LIST_DIR}/*.cpp
)

idf_component_register(
    SRCS ${ARDUINO_FOC_SRC}
    INCLUDE_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/src
    REQUIRES
    ArduinoCore-avr-1.8.7
)
