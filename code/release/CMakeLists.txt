add_executable(release main.c ball.cpp common.c bmi270.c bmi2.c)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)
pico_generate_pio_header(release ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

# pull in common dependencies
target_link_libraries(release pico_stdlib hardware_i2c hardware_gpio pico_multicore hardware_pio)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(release)

# enable usb output, disable uart output
pico_enable_stdio_usb(release 1)
pico_enable_stdio_uart(release 0)