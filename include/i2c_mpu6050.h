/*
 * Copyright (c) 2024 xtedious
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef _I2C_MPU6050_H_
#define _I2C_MPU6050_H_

#include "hardware/i2c.h"

#include <stdbool.h>
#include <stdint.h>

// MPU6050 Registers
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define FIFO_EN 0x23
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A
#define FIFO_COUNTH 0x72
#define FIFO_COUNTL 0x73
#define FIFO_R_W 0x74
#define USER_CTRL 0x6A
#define INT_PIN_CFG 0x37

#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define TEMP_OUT_H 0x41
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

#define DELTA_T 0.1f // delta_t in seconds(used for integration calculation)

// MPU6050 Configuration Parameters
typedef enum {
    ACCEL_RANGE_2G = 0,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G
} ACCEL_RANGE;

typedef enum {
    GYRO_SCALE_250DPS = 0,
    GYRO_SCALE_500DPS,
    GYRO_SCALE_1000DPS,
    GYRO_SCALE_2000DPS
} GYRO_SCALE;

typedef enum {
    FIFO_ACCEL_GYRO_TEMP = 0,
    FIFO_ACCEL_GYRO,
    FIFO_ACCEL,
    FIFO_GYRO,
    FIFO_TEMP
} FIFO_CONFIG;

typedef enum {
    DLPF_CFG_0 = 0, // 260Hx Bandwidth
    DLPF_CFG_1,     // 184Hz Bandwidth
    DLPF_CFG_2,     // 94Hz Bandwidth
    DLPF_CFG_3,     // 44Hz Bandwidth
    DLPF_CFG_4,     // 21Hz Bandwidth
    DLPF_CFG_5,     // 10Hz Bandwidth
    DLPF_CFG_6,     // 5Hz Bandwidth
} DLPF_CONFIG;

typedef struct {
        ACCEL_RANGE accel_range;
        GYRO_SCALE gyro_scale;
        DLPF_CONFIG dlpf_cfg;
        FIFO_CONFIG fifo_config;
        int sample_rate;
        bool fifo_en;
        bool fifo_gyro;
        bool fifo_accel;
        bool fifo_temp;
} mpu6050_config;

typedef struct {
        int16_t rawAccel[3];
        int16_t rawGyro[3];
        int16_t rawTemp;
        float accelX, accelY, accelZ;
        float gyroX, gyroY, gyroZ;
        // signed angle from the normal vecto of the sensor,
        // Note yaw is a signed angle so direction of rotation can be determind
        float roll, pitch, yaw;
        // signed area of the data drom the gyroscope, gyroZ integral(yaw) is
        // used for yaw calculation Ouput is in degrees not radians. Only
        // usefull for knowing signed deviation from the imu level Call
        // mpu6050_run_euler_calibration() to minimise error in calculation.
        float int_roll, int_pitch;
        float tempC; // This is the temperature in degree Celsius
        float gyroX_error, gyroY_error, gyroZ_error;
} mpu6050_data;

typedef struct {
        i2c_inst_t *i2c_inst;
        uint8_t addr;
        mpu6050_config *config;
        float accel_lsb, gyro_lsb;
        uint sda_pin, scl_pin;
} mpu6050_device;

// Function prototypes
// mpu6050 Functions
void mpu6050_init(mpu6050_device *device);
void mpu6050_set_gyro_scale(mpu6050_device *device);
void mpu6050_set_accel_range(mpu6050_device *device);
void mpu6050_set_fifo_config(mpu6050_device *device);
void mpu6050_set_sample_rate_div(mpu6050_device *device);

// TO DO -
void mpu6050_set_dlpf_bandwidth(mpu6050_device *device);
void mpu6050_fifo_read(mpu6050_device *device, mpu6050_data *sensor_data);

void mpu6050_poll_data(mpu6050_device *device, mpu6050_data *sensor_data);

// Generates yaw, pitch and roll numbers
void mpu6050_gen_euler_angles(mpu6050_device *device,
                              mpu6050_data *sensor_data);

// Prints all the data comming from the mpu6050 (accel in g, gyro in dps adn
// temp in degree Celsius)
void mpu6050_print_imu_data(mpu6050_device *device, mpu6050_data *sensor_data);

// Prints yaw, pitch and roll, passing true in the last parameter will print
// calculated gyro errors and signed_roll and signed_pitch.
void mpu6050_print_euler_angles(mpu6050_device *device,
                                mpu6050_data *sensor_data, bool full);

void mpu6050_run_euler_calibration(mpu6050_device *device,
                                   mpu6050_data *sensor_data);

// This function sets up the mpu6050 to the default i2c instance which is on
// gpio// 4 and 5, it also initializes the i2c to 400kHz
mpu6050_device *mpu6050_default_config();

mpu6050_device *mpu6050_set_config(i2c_inst_t *i2c_instance, uint gpio_sda,
                                   uint gpio_scl, uint8_t device_address,
                                   ACCEL_RANGE accel, GYRO_SCALE gyro,
                                   int device_sample_rate, bool fifo);

// General Functions
// Estimate the gyro error for later integration use
float estimate_gyro_error(mpu6050_device *device);

#endif // _I2C_MPU6050_H_
