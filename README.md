# pico_i2c_mpu6050
A library to use the mpu6050 on the pico with i2c

Just copy these files into whatever project want them in and include the i2c_mpu6050.c file in the add_executables() in the CMakeList.txt

To do list is ordered (order is priority)
## TO DO:
- implement print debuging to diagnose issues
- fix the error in the integrated roll and pitch angles
- implement proper error handling and a few edge cases
- Properly make it into a library with add_library()
- Allow reading from the FIFO buffer on the mpu6050
- Implement Kalmann filter
- Generate rotation quaternions
