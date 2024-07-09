To work towards a more sustainable future in software development, ive decided to start keeping these readme logs. I just spent a summer at an org with horribly documented code and its made me want to change mine as well.

# File structure
**release** is the current working model deployed on the device

### The following functions are all from the [BMI270 api by bosch sensor tech](https://github.com/boschsensortec/BMI270_SensorAPI)
- `bmi2_defs.h`
- `bmi2.c/h`
- `bmi270.c/h`
  
`common.c/h` holds the interface functions that enable the rp2040 to use the api

### My functions were:

- `main.cpp` holds the main method - performing all system initialization on multithreading / repeated timers
- `imu.cpp` abstracts away most of the boilerplate imu code.



# Dependencies

BMI270 library from bosch sensor tech


# Troubleshooting Notes
Note that the RP2040 cannot do I2C writes greater than ~128 bytes, so the 8kb config file command is incompatible. 