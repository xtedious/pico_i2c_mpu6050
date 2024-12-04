#include <stdio.h>

#include "../include/i2c_mpu6050.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    i2c_init(i2c0, 400 * 1000);

    mpu6050_data imu_data;

    mpu6050_device *mpu6050 =
        mpu6050_set_config(i2c0, 4, 5, MPU6050_ADDR, ACCEL_RANGE_2G,
                           GYRO_SCALE_250DPS, 1000, false);

    mpu6050_init(mpu6050);
    mpu6050_run_euler_calibration(mpu6050, &imu_data);

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    while (1) {
        gpio_put(25, !gpio_get(25));
        sleep_ms(DELTA_T * 1000);
        mpu6050_print_imu_data(mpu6050, &imu_data);
        mpu6050_print_euler_angles(mpu6050, &imu_data, true);
    }
}
