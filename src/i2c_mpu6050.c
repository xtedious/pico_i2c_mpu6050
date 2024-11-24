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

#include "../include/i2c_mpu6050.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <math.h>
#include <stdio.h>

// ACCEL and GYRO Settings
#define ACCEL_2G 0x00
#define ACCEL_4G 0x01
#define ACCEL_8G 0x02
#define ACCEL_16G 0x03

// LSB Sensitivity for Sensors
#define ACCEL_2G_LSB 16384.0f
#define ACCEL_4G_LSB 8192.0f
#define ACCEL_8G_LSB 4092.0f
#define ACCEL_16G_LSB 2048.0f
#define GYRO_250DPS_LSB 131.0f
#define GYRO_500DPS_LSB 65.5f
#define GYRO_1000DPS_LSB 32.8f
#define GYRO_2000DPS_LSB 16.4f

#define GYRO_OUT_RATE_DLPF_OFF 8000
#define GYRO_OUT_RATE_DLPF_ON 1000
// FIFO DATA configs (in this case A accelerometer, G gyroscope, T temperature)
#define FIFO_A_G_T 0xF8
#define FIFO_A_G 0x78
#define FIFO_A 0x08
#define FIFO_G 0x70
#define FIFO_T 0x80

#define FIFO_EN_VAL 0x40
#define FIFO_RESET_VAL 0x04
#define DATA_RDY_EN 0x01

// Bandwidth in Hz
#define DLPF_CFG_0_RATE 260.0f
#define DLPF_CFG_1_RATE 184.0f
#define DLPF_CFG_2_RATE 94.0f
#define DLPF_CFG_3_RATE 44.0f
#define DLPF_CFG_4_RATE 21.0f
#define DLPF_CFG_5_RATE 10.0f
#define DLPF_CFG_6_RATE 5.0f

#define GYRO_ERROR_ITERATION 50

mpu6050_config default_config = {.gyro_scale = GYRO_SCALE_250DPS,
                                 .accel_range = ACCEL_RANGE_2G,
                                 .fifo_en = false,
                                 .sample_rate = 1000};

static int mpu6050_write_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg,
                             uint8_t value) {
    uint8_t buff[2] = {reg, value};
    return i2c_write_blocking(i2c, addr, buff, 2, false);
}

static int mpu6050_read_reg(i2c_inst_t *i2c, uint8_t addr, uint8_t reg,
                            uint8_t *buffer, size_t length) {
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    return i2c_read_blocking(i2c, addr, buffer, length, false);
}

// Initialize the mpu6050 with sensor configs
void mpu6050_init(mpu6050_device *device) {
    // Reset MPU6050
    mpu6050_write_reg(device->i2c_inst, device->addr, PWR_MGMT_1, 0x80);
    sleep_ms(200);

    // Clear Reset write
    mpu6050_write_reg(device->i2c_inst, device->addr, PWR_MGMT_1, 0x00);
    sleep_ms(200);

    mpu6050_set_sample_rate_div(device);
    mpu6050_set_accel_range(device);
    mpu6050_set_gyro_scale(device);

    if (device->config->fifo_en) {
        mpu6050_set_fifo_config(device);
    }

    if (device->config == NULL) {
        device->config = &default_config;
    }
}

mpu6050_device *mpu6050_default_config() {
    static mpu6050_device device;
    device.config = &default_config;
    device.addr = MPU6050_ADDR;
    device.i2c_inst = i2c0;

    return &device;
}

mpu6050_device *mpu6050_set_config(i2c_inst_t *i2c_instance,
                                   uint8_t device_address, ACCEL_RANGE accel,
                                   GYRO_SCALE gyro, int device_sample_rate,
                                   bool fifo) {
    static mpu6050_device device;
    static mpu6050_config conf;

    device.config = &conf;

    device.addr = device_address;
    device.i2c_inst = i2c_instance;

    device.config->sample_rate = device_sample_rate;
    device.config->accel_range = ACCEL_RANGE_2G;
    device.config->gyro_scale = GYRO_SCALE_250DPS;

    device.config->fifo_en = fifo;

    return &device;
}

void mpu6050_set_accel_range(mpu6050_device *device) {
    switch (device->config->accel_range) {
    case ACCEL_RANGE_2G:
        mpu6050_write_reg(device->i2c_inst, device->addr, ACCEL_CONFIG, 0x00);
        device->accel_lsb = ACCEL_2G_LSB;
        break;
    case ACCEL_RANGE_4G:
        mpu6050_write_reg(device->i2c_inst, device->addr, ACCEL_CONFIG, 0x01);
        device->accel_lsb = ACCEL_4G_LSB;
        break;
    case ACCEL_RANGE_8G:
        mpu6050_write_reg(device->i2c_inst, device->addr, ACCEL_CONFIG, 0x02);
        device->accel_lsb = ACCEL_8G_LSB;
        break;
    case ACCEL_RANGE_16G:
        mpu6050_write_reg(device->i2c_inst, device->addr, ACCEL_CONFIG, 0x03);
        device->accel_lsb = ACCEL_16G_LSB;
        break;
    default:
        break;
    }
}

void mpu6050_set_gyro_scale(mpu6050_device *device) {
    switch (device->config->gyro_scale) {
    case GYRO_SCALE_250DPS:
        mpu6050_write_reg(device->i2c_inst, device->addr, GYRO_CONFIG, 0x00);
        device->gyro_lsb = GYRO_250DPS_LSB;
        break;
    case GYRO_SCALE_500DPS:
        mpu6050_write_reg(device->i2c_inst, device->addr, GYRO_CONFIG, 0x01);
        device->gyro_lsb = GYRO_500DPS_LSB;
    case GYRO_SCALE_1000DPS:
        mpu6050_write_reg(device->i2c_inst, device->addr, GYRO_CONFIG, 0x02);
        device->gyro_lsb = GYRO_1000DPS_LSB;
        break;
    case GYRO_SCALE_2000DPS:
        mpu6050_write_reg(device->i2c_inst, device->addr, GYRO_CONFIG, 0x03);
        device->gyro_lsb = GYRO_2000DPS_LSB;
    default:
        break;
    }
}

// TO_DO - remove enums and use enable flags/parameters
void mpu6050_set_fifo_config(mpu6050_device *device) {}

void mpu6050_set_sample_rate_div(mpu6050_device *device) {

    float gyro_output_rate = device->config->dlpf_cfg == DLPF_CFG_0
                                 ? GYRO_OUT_RATE_DLPF_ON
                                 : GYRO_OUT_RATE_DLPF_OFF;

    float sample_rate_div =
        (gyro_output_rate / (float)device->config->sample_rate) - 1;
    mpu6050_write_reg(device->i2c_inst, device->addr, SMPLRT_DIV,
                      (uint8_t)sample_rate_div);
}

void mpu6050_fifo_read(mpu6050_device *device, mpu6050_data *sensor_data) {
    // TO_DO
}

void mpu6050_poll_data(mpu6050_device *device, mpu6050_data *sensor_data) {
    uint8_t buffer[6];

    mpu6050_read_reg(device->i2c_inst, device->addr, ACCEL_XOUT_H, buffer, 6);
    for (int i = 0; i < 3; i++) {
        sensor_data->rawAccel[i] =
            (int16_t)(buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    mpu6050_read_reg(device->i2c_inst, device->addr, GYRO_XOUT_H, buffer, 6);
    for (int i = 0; i < 3; i++) {
        sensor_data->rawGyro[i] =
            (int16_t)(buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    mpu6050_read_reg(device->i2c_inst, device->addr, TEMP_OUT_H, buffer, 2);
    sensor_data->rawTemp = (int16_t)(buffer[0] << 8 | buffer[1]);
}

static void mpu6050_process_raw(mpu6050_device *device,
                                mpu6050_data *sensor_data) {
    sensor_data->accelX = sensor_data->rawAccel[0] / device->accel_lsb;
    sensor_data->accelY = sensor_data->rawAccel[1] / device->accel_lsb;
    sensor_data->accelZ = sensor_data->rawAccel[2] / device->accel_lsb;

    sensor_data->gyroX = sensor_data->rawGyro[0] / device->gyro_lsb;
    sensor_data->gyroY = sensor_data->rawGyro[1] / device->gyro_lsb;
    sensor_data->gyroZ = sensor_data->rawGyro[2] / device->gyro_lsb;

    // Conversion factor of raw temp data from imu
    sensor_data->tempC = (float)sensor_data->rawTemp / 340 + 36.53;
}

void mpu6050_gen_euler_angles(mpu6050_device *device,
                              mpu6050_data *sensor_data) {
    mpu6050_poll_data(device, sensor_data);
    mpu6050_process_raw(device, sensor_data);

    sensor_data->roll =
        acos(sensor_data->accelZ /
             sqrt(pow(sensor_data->accelY, 2) + pow(sensor_data->accelZ, 2))) *
        180.0f / M_PI;
    sensor_data->pitch =
        acos(sensor_data->accelZ /
             sqrt(pow(sensor_data->accelX, 2) + pow(sensor_data->accelZ, 2))) *
        180.0f / M_PI;
    sensor_data->yaw =
        sensor_data->yaw +
        ((sensor_data->gyroZ - sensor_data->gyroZ_error) * DELTA_T);
}

void mpu6050_print_euler_angles(mpu6050_device *device,
                                mpu6050_data *sensor_data) {
    mpu6050_gen_euler_angles(device, sensor_data);

    printf("roll: %f degrees\t pitch: %f degrees\t yaw: %f degrees\n "
           "yaw_error: %f degrees\n",
           sensor_data->roll, sensor_data->pitch, sensor_data->yaw,
           sensor_data->gyroZ_error);
}

void mpu6050_print_imu_data(mpu6050_device *device, mpu6050_data *sensor_data) {

    mpu6050_poll_data(device, sensor_data);

    mpu6050_process_raw(device, sensor_data);
    printf("accelX: %f g\t accelY: %f g\t accelZ: %f g\n temp: %f C\n",
           sensor_data->accelX, sensor_data->accelY, sensor_data->accelZ,
           sensor_data->tempC);
    printf("gyroX: %f dps\t gyroY: %f dps\t gyroZ: %f dps\n",
           sensor_data->gyroX, sensor_data->gyroY, sensor_data->gyroZ);
}

void mpu6050_run_euler_calibration(mpu6050_device *device,
                                   mpu6050_data *sensor_data) {

    sensor_data->gyroZ_error = estimate_gyroZ_error(device);
}

float estimate_gyroZ_error(mpu6050_device *device) {
    mpu6050_data temp_data;

    float totalError = 0;
    for (int i = 0; i < GYRO_ERROR_ITERATION; i++) {
        mpu6050_poll_data(device, &temp_data);
        mpu6050_process_raw(device, &temp_data);

        totalError += temp_data.gyroZ;

        printf("\rProgress: %d/%d Error Iterations", i, GYRO_ERROR_ITERATION);
        fflush(stdout);
        sleep_ms(50);
    }

    return totalError / GYRO_ERROR_ITERATION;
}
