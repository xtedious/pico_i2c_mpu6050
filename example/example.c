#include <stdio.h>

#include "../include/i2c_mpu6050.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    i2c_init(i2c0, 400 * 1000);
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    mpu6050_data imu_data;

    mpu6050_device *mpu6050 = mpu6050_set_config(
        i2c0, MPU6050_ADDR, ACCEL_RANGE_2G, GYRO_SCALE_250DPS, 1000, false);

    /*mpu6050_device *mpu6050 = mpu6050_default_config();*/

    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    mpu6050_init(mpu6050);

    while (1) {
        gpio_put(25, !gpio_get(25));
        sleep_ms(100);
        mpu6050_print_imu_data(mpu6050, &imu_data);
    }
}
