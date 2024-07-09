/**\
 * Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "bmi270.h"
#include "common.h"

#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"



#define IS_RGBW false
#define NUM_PIXELS 25
#define WS2812_PIN 16
uint32_t leds[NUM_PIXELS];

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

static inline void fill_strip(uint32_t* leds,  uint32_t numLeds) {
    for(int i = 0; i < numLeds; i++) {
        put_pixel(leds[i]);
    }
}


/******************************************************************************/
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*! Macros to select the sensors                   */
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)

/******************************************************************************/
/*!           Static Function Declaration                                     */

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] bmi       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi);

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Accel values in meter per second squared.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

typedef struct Motion {
    float ax;
    float ay;
    float az;
    float gx;
    float gy; 
    float gz;
} Motion;

Motion* initMotion() {
    Motion* move = malloc(sizeof(Motion));
    memset(move, 0x00, sizeof(Motion));
    return move;
}

void deinitMotion(Motion* move) {
    free(move);
}

Motion* data;



/******************************************************************************/
/*!            Functions                                        */

/* This function starts the execution of program. */
void core1_entry()
{
    // while(1) {
    //     printf("helo from core 1;\n");
    // }
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Assign accel and gyro sensor to variable. */
    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    /* Sensor initialization configuration. */
    struct bmi2_dev bmi;

    /* Structure to define type of sensor and their respective data. */
    struct bmi2_sens_data sensor_data = { { 0 } };

    struct bmi2_sens_config config;

    /* Interface reference is given as a parameter
     * For I2C : BMI2_I2C_INTF
     * For SPI : BMI2_SPI_INTF
     */
    rslt = bmi2_interface_init(&bmi, 1);
    bmi2_error_codes_print_result(rslt);

    /* Initialize bmi270. */
    rslt = bmi270_init(&bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* Accel and gyro configuration settings. */
        rslt = set_accel_gyro_config(&bmi);
        bmi2_error_codes_print_result(rslt);

        if (rslt == BMI2_OK)
        {
            /* NOTE:
             * Accel and Gyro enable must be done after setting configurations
             */
            rslt = bmi2_sensor_enable(sensor_list, 2, &bmi);
            bmi2_error_codes_print_result(rslt);

            if (rslt == BMI2_OK)
            {
                config.type = BMI2_ACCEL;

                /* Get the accel configurations. */
                rslt = bmi2_get_sensor_config(&config, 1, &bmi);
                bmi2_error_codes_print_result(rslt);

                printf(
                    "\nData set, Accel Range, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_ms2_X, Acc_ms2_Y, Acc_ms2_Z, Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyro_DPS_X, Gyro_DPS_Y, Gyro_DPS_Z\n\n");

                while (1)
                {
                    rslt = bmi2_get_sensor_data(&sensor_data, &bmi);
                    // bmi2_error_codes_print_result(rslt);
                    // printf("%02x\n", (&config)[ACCEL].cfg.acc.odr);
                    if ((rslt == BMI2_OK) && (sensor_data.status & BMI2_DRDY_ACC) &&
                        (sensor_data.status & BMI2_DRDY_GYR))
                    {
                        /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                        data->ax = lsb_to_mps2(sensor_data.acc.x, (float)2, bmi.resolution);
                        data->ay = lsb_to_mps2(sensor_data.acc.y, (float)2, bmi.resolution);
                        data->az = lsb_to_mps2(sensor_data.acc.z, (float)2, bmi.resolution);

                        /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                        data->gx = lsb_to_dps(sensor_data.gyr.x, (float)2000, bmi.resolution);
                        data->gy = lsb_to_dps(sensor_data.gyr.y, (float)2000, bmi.resolution);
                        data->gz = lsb_to_dps(sensor_data.gyr.z, (float)2000, bmi.resolution);

                        // printf("%d, %d, %d, %d, %d, %4.2f, %4.2f, %4.2f, %d, %d, %d, %4.2f, %4.2f, %4.2f\n",
                        //        indx,
                        //        config.cfg.acc.range,
                        //        sensor_data.acc.x,
                        //        sensor_data.acc.y,
                        //        sensor_data.acc.z,
                        //        acc_x,
                        //        acc_y,
                        //        acc_z,
                        //        sensor_data.gyr.x,
                        //        sensor_data.gyr.y,
                        //        sensor_data.gyr.z,
                        //        gyr_x,
                        //        gyr_y,
                        //        gyr_z);

                    }
                }
            }
        }
    }

    // return rslt;
}


uint8_t map_accel_to_uint8(float a) {
    a = abs(a);
    return (uint8_t)(a/15*255);
}

uint8_t map_gyro_to_uint8(float a) {
    a = abs(a);
    return (uint8_t)(a/40*255);
}

int main(void) {
    stdio_init_all();

    data = initMotion();
    multicore_launch_core1(core1_entry);
    

    //led stuff
    memset(leds, 0x05, sizeof(uint32_t)*NUM_PIXELS);



    PIO pio = pio0;
    int sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);

    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);


    while(1) {
        // printf("Motion: %f, %f, %f, %f, %f, %f\n", data->ax, data->ay, data->az, data->gx, data->gy, data->gz);
        leds[0] = urgb_u32(map_accel_to_uint8(data->ax), 0, 0);
        leds[1] = urgb_u32(map_accel_to_uint8(data->ay), 0, 0);
        leds[2] = urgb_u32(map_accel_to_uint8(data->az), 0, 0);
        leds[5] = urgb_u32(0, map_gyro_to_uint8(data->gx), 0);
        leds[6] = urgb_u32(0, map_gyro_to_uint8(data->gy), 0);
        leds[7] = urgb_u32(0, map_gyro_to_uint8(data->gz), 0);
        fill_strip(leds, NUM_PIXELS);
        sleep_ms(1);
    }
    
    deinitMotion(data);
}


/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);
    bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);
    bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_1600HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi2_set_sensor_config(config, 2, bmi);
        bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}
