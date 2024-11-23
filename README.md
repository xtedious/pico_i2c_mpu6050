# pico_i2c_mpu6050
A library to use the mpu6050 on the pico with i2c

Just copy these files into whatever project want them in and include the i2c_mpu6050.c file in the add_executables() in the CMakeList.txt

TO DO:
- Allow reading from the FIFO buffer on the mpu6050
- Flesh out the comments on the functions
- Properly make it into a library with add_library()
- Write the yaw measurement using integration
- Generate rotation quaternions

## License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.
