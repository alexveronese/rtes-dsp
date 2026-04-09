# Define the executable for the Pico Micro-ROS example
add_executable(rtes-dsp
    main.c
)

# Link against Pico, FreeRTOS, microros, and shared utilities
target_link_libraries(rtes-dsp
    pico_stdlib
    FreeRTOS-Kernel FreeRTOS-Kernel-Heap4
    microros
    common_utils
    hardware_adc
    hardware_uart
    hardware_timer
)

# Configure Pico IO settings
pico_enable_stdio_usb(rtes-dsp 1)
pico_enable_stdio_uart(rtes-dsp 0)


# Compile definitions for Pico config
target_compile_definitions(rtes-dsp PRIVATE
    PICO_UART_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
)

# Set UF2 output directory
set_target_properties(rtes-dsp PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Generate UF2 and binary outputs
pico_add_extra_outputs(rtes-dsp)
