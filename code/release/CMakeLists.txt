add_executable(release main.cpp ball.cpp)

# pull in common dependencies
target_link_libraries(release pico_stdlib hardware_i2c hardware_gpio)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(release)

# enable usb output, disable uart output
pico_enable_stdio_usb(release 1)
pico_enable_stdio_uart(release 0)