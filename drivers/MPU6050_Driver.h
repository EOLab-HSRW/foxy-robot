
#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include <jetgpio.h>
#include <unistd.h>

class MPU6050 {
private:
    
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


    int i2cHandle;
    bool initialized;

    // Internal function for setting up MPU6050 registers
    int setupRegisters();

public:
    MPU6050();
    ~MPU6050();

    // Initialize the sensor
    int initialize();

    // Read gyro values
    int readGyro(float &gyro_x, float &gyro_y, float &gyro_z);

    // Read accelerometer values
    int readAccel(float &accel_x, float &accel_y, float &accel_z);

    // Terminate the connection and cleanup
    void terminate();
};

#endif
