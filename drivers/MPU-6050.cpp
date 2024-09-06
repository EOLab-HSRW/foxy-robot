#include <iostream>
#include <unistd.h>
#include <jetgpio.h>

/* MPU-6050 Registers */
const int MPU6050_SLAVE_ADDRESS = 0x68;
const int PWR_MGMT_1 = 0x6B;
const int ACCEL_CONFIG = 0x1C;
const int ACCEL_RANGE_4G = 0x08;
const int GYRO_CONFIG = 0x1B;
const int GYRO_RANGE_250DEG = 0x00;
const float GYRO_SCALE_MODIFIER_250DEG = 131.0; // value from the MPU datashee
const float ACCEL_SCALE_MODIFIER_4G = 8192.0;  // value from the MPU datasheet
const int GYRO_XOUT0 = 0x43;
const int GYRO_YOUT0 = 0x45;
const int GYRO_ZOUT0 = 0x47;
const int ACCEL_XOUT0 = 0x3B;
const int ACCEL_YOUT0 = 0x3D;
const int ACCEL_ZOUT0 = 0x3F;

/* Function to combine high and low bytes into a signed 16-bit value, similar to that of the Jetgpio_i2c*/
int16_t combineBytes(uint8_t high, uint8_t low) {
    int16_t value = (high << 8) | low;
    if (value >= 0x8000) {
        value = -(65535 - value + 1);
    }
    return value;
}

int main() {
    int Init;
    
    // Gyroscope and Accelerometer variables initialize to 0's
    float gyro_x = 0, gyro_y = 0, gyro_z = 0;
    float accel_x = 0, accel_y = 0, accel_z = 0;

    // Jetgpio Initialization
    Init = gpioInitialise();
    if (Init < 0) {
        /* jetgpio initialization failed */
        std::cerr << "Jetgpio initialization failed with code: " << Init << std::endl;
        exit(Init);
    } else {
        /* jetgpio initialization okay */
        std::cout << "Jetgpio initialized successfully." << std::endl;
    }

    // Openning the connection to the i2c MPU-6050 on bus 0
    int MPU6050 = i2cOpen(0, MPU6050_SLAVE_ADDRESS);
    if (MPU6050 < 0) {
        /* Problem encountered openning the i2c port */
        std::cerr << "Failed to open I2C connection with MPU6050, error code: " << MPU6050 << std::endl;
        gpioTerminate();
        return -1;
    }
    /* Openning i2c port ok */
    std::cout << "I2C connection to MPU6050 opened successfully." << std::endl;

    // Wake up MPU-6050 (it starts in sleep mode)
    i2cWriteByteData(MPU6050, PWR_MGMT_1, 0x00);
    usleep(100000);

    // Set up Accelerometer range to 4G
    i2cWriteByteData(MPU6050, ACCEL_CONFIG, ACCEL_RANGE_4G);
    usleep(100000);

    // Set up Gyroscope range to 250 deg/sec
    i2cWriteByteData(MPU6050, GYRO_CONFIG, GYRO_RANGE_250DEG);
    usleep(100000);

    // Main loop to read gyroscope and accelerometer data
    for (int i = 0; i < 1000; ++i) {
        // Read gyroscope values
        uint8_t gyro_x_H = i2cReadByteData(MPU6050, GYRO_XOUT0);
        uint8_t gyro_x_L = i2cReadByteData(MPU6050, GYRO_XOUT0 + 1);
        gyro_x = combineBytes(gyro_x_H, gyro_x_L) / GYRO_SCALE_MODIFIER_250DEG;

        uint8_t gyro_y_H = i2cReadByteData(MPU6050, GYRO_YOUT0);
        uint8_t gyro_y_L = i2cReadByteData(MPU6050, GYRO_YOUT0 + 1);
        gyro_y = combineBytes(gyro_y_H, gyro_y_L) / GYRO_SCALE_MODIFIER_250DEG;

        uint8_t gyro_z_H = i2cReadByteData(MPU6050, GYRO_ZOUT0);
        uint8_t gyro_z_L = i2cReadByteData(MPU6050, GYRO_ZOUT0 + 1);
        gyro_z = combineBytes(gyro_z_H, gyro_z_L) / GYRO_SCALE_MODIFIER_250DEG;

        // Read accelerometer values
        uint8_t accel_x_H = i2cReadByteData(MPU6050, ACCEL_XOUT0);
        uint8_t accel_x_L = i2cReadByteData(MPU6050, ACCEL_XOUT0 + 1);
        accel_x = combineBytes(accel_x_H, accel_x_L) / ACCEL_SCALE_MODIFIER_4G;

        uint8_t accel_y_H = i2cReadByteData(MPU6050, ACCEL_YOUT0);
        uint8_t accel_y_L = i2cReadByteData(MPU6050, ACCEL_YOUT0 + 1);
        accel_y = combineBytes(accel_y_H, accel_y_L) / ACCEL_SCALE_MODIFIER_4G;

        uint8_t accel_z_H = i2cReadByteData(MPU6050, ACCEL_ZOUT0);
        uint8_t accel_z_L = i2cReadByteData(MPU6050, ACCEL_ZOUT0 + 1);
        accel_z = combineBytes(accel_z_H, accel_z_L) / ACCEL_SCALE_MODIFIER_4G;

        // Output values for debugging
        std::cout << "Gyro X: " << gyro_x << " | Gyro Y: " << gyro_y << " | Gyro Z: " << gyro_z << std::endl;
        std::cout << "Accel X: " << accel_x << " | Accel Y: " << accel_y << " | Accel Z: " << accel_z << std::endl;
        usleep(10000);  
    }

    // Close the I2C connection
    i2cClose(MPU6050);
    gpioTerminate();
    return 0;
}
