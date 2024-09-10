#include "MPU6050_Driver.h"
#include <iostream>
#include <jetgpio.h>

// Constructor Definition
MPU6050::MPU6050() : i2cHandle(-1), initialized(false) {}

MPU6050::~MPU6050() {
    terminate();
}

int MPU6050::setupRegisters() {
    // Wake up the MPU-6050 (starts in sleep mode)
    int statusCode = i2cWriteByteData(i2cHandle, PWR_MGMT_1, 0x00);
    if (statusCode < 0) return -1;

    usleep(100000);

    // Set accelerometer range to 4G
    statusCode = i2cWriteByteData(i2cHandle, ACCEL_CONFIG, ACCEL_RANGE_4G);
    if (statusCode < 0) return -1;

    usleep(100000);

    // Set gyroscope range to 250 degrees/second
    statusCode = i2cWriteByteData(i2cHandle, GYRO_CONFIG, GYRO_RANGE_250DEG);
    if (statusCode < 0) return -1;

    usleep(100000);

    return 0;
}

int MPU6050::initialize() {
    if (initialized) return 0;

    // Initialize GPIO/I2C interface
    if (gpioInitialise() < 0) {
        return -1;
    }

    // Open I2C connection with the MPU6050
    i2cHandle = i2cOpen(0, MPU6050_SLAVE_ADDRESS);
    if (i2cHandle < 0) {
        return -1;
    }

    // Setup MPU6050 configuration registers
    if (setupRegisters() < 0) {
        return -1;
    }

    initialized = true;
    return 0;
}

int MPU6050::readGyro(float &gyro_x, float &gyro_y, float &gyro_z) {
    if (!initialized) return -1;

    int gyro_x_H = i2cReadByteData(i2cHandle, GYRO_XOUT0);
    int gyro_x_L = i2cReadByteData(i2cHandle, GYRO_XOUT0 + 1);
    gyro_x = (gyro_x_H << 8) + gyro_x_L;
    if (gyro_x >= 0x8000) gyro_x = -(65535 - gyro_x) + 1;
    gyro_x /= GYRO_SCALE_MODIFIER_250DEG;

    int gyro_y_H = i2cReadByteData(i2cHandle, GYRO_YOUT0);
    int gyro_y_L = i2cReadByteData(i2cHandle, GYRO_YOUT0 + 1);
    gyro_y = (gyro_y_H << 8) + gyro_y_L;
    if (gyro_y >= 0x8000) gyro_y = -(65535 - gyro_y) + 1;
    gyro_y /= GYRO_SCALE_MODIFIER_250DEG;

    int gyro_z_H = i2cReadByteData(i2cHandle, GYRO_ZOUT0);
    int gyro_z_L = i2cReadByteData(i2cHandle, GYRO_ZOUT0 + 1);
    gyro_z = (gyro_z_H << 8) + gyro_z_L;
    if (gyro_z >= 0x8000) gyro_z = -(65535 - gyro_z) + 1;
    gyro_z /= GYRO_SCALE_MODIFIER_250DEG;

    return 0;
}

int MPU6050::readAccel(float &accel_x, float &accel_y, float &accel_z) {
    if (!initialized) return -1;

    int accel_x_H = i2cReadByteData(i2cHandle, ACCEL_XOUT0);
    int accel_x_L = i2cReadByteData(i2cHandle, ACCEL_XOUT0 + 1);
    accel_x = (accel_x_H << 8) + accel_x_L;
    if (accel_x >= 0x8000) accel_x = -(65535 - accel_x) + 1;
    accel_x /= ACCEL_SCALE_MODIFIER_4;

    int accel_y_H = i2cReadByteData(i2cHandle, ACCEL_YOUT0 );
    int accel_y_L = i2cReadByteData(i2cHandle, ACCEL_YOUT0 + 1);
    accel_y = (accel_y_H << 8) + accel_y_L;
    if (accel_y >= 0x8000) accel_y = -(65535 - accel_y) + 1;
    accel_y /= ACCEL_SCALE_MODIFIER_4;

    int accel_z_H = i2cReadByteData(i2cHandle, ACCEL_ZOUT0);
    int accel_z_L = i2cReadByteData(i2cHandle, ACCEL_ZOUT0 + 1);
    accel_z = (accel_z_H << 8) + accel_z_L;
    if (accel_z >= 0x8000) accel_z = -(65535 - accel_z) + 1;
    accel_z /= ACCEL_SCALE_MODIFIER_4;

    return 0;
}

void MPU6050::terminate() {
    if (i2cHandle >= 0) {
        i2cClose(i2cHandle);
        gpioTerminate();
        initialized = false;
    }
}
