#include <iostream>
#include <cstdint>
#include <unistd.h>
#include <jetgpio.h>

class MPU6050 {
private:
    int i2cHandle;
    const int MPU6050_SLAVE_ADDRESS = 0x68;
    const int PWR_MGMT_1 = 0x6B;
    const int ACCEL_CONFIG = 0x1C;
    const int ACCEL_RANGE_4G = 0x08;
    const int GYRO_CONFIG = 0x1B;
    const int GYRO_RANGE_250DEG = 0x00;
    const float GYRO_SCALE_MODIFIER_250DEG = 131.0;
    const int GYRO_XOUT0 = 0x43;
    const int GYRO_YOUT0 = 0x45;
    const int GYRO_ZOUT0 = 0x47;

public:
    MPU6050() {
        // Initialize JETGPIO
        int Init = gpioInitialise();
        if (Init < 0) {
            std::cerr << "JETGPIO initialization failed with error code: " << Init << std::endl;
            exit(Init);
        } else {
            std::cout << "JETGPIO initialized successfully with return code: " << Init << std::endl;
        }

        // Opens the I2C connection to MPU6050
        i2cHandle = i2cOpen(0, MPU6050_SLAVE_ADDRESS);
        if (i2cHandle < 0) {
            std::cerr << "Failed to open I2C with error code: " << i2cHandle << std::endl;
            gpioTerminate();
            exit(i2cHandle);
        } else {
            std::cout << "I2C connection opened successfully. Handler: " << i2cHandle << std::endl;
        }

        // Wake up MPU6050
        writeRegister(PWR_MGMT_1, 0x00);
        usleep(100000);

        // Set the accelerometer range to 4G
        writeRegister(ACCEL_CONFIG, ACCEL_RANGE_4G);
        usleep(100000);

        // Set the gyroscope range to 250 deg/second
        writeRegister(GYRO_CONFIG, GYRO_RANGE_250DEG);
        usleep(100000);
    }

    ~MPU6050() {
        // Close I2C connection
        int i2cCloseStatus = i2cClose(i2cHandle);
        if (i2cCloseStatus >= 0) {
            std::cout << "I2C connection closed successfully." << std::endl;
        } else {
            std::cerr << "Failed to close I2C connection with error code: " << i2cCloseStatus << std::endl;
        }

        // Terminate GPIO
        gpioTerminate();
    }

    // Function (write a byte of data to a specific register)
    void writeRegister(int reg, int data) {
        int writeStatus = i2cWriteByteData(i2cHandle, MPU6050_SLAVE_ADDRESS, reg, data);
        if (writeStatus < 0) {
            std::cerr << "Failed to write to register: " << reg << " Error code: " << writeStatus << std::endl;
        }
    }

    // Function (read gyroscope values from the MPU-6050)
    float readGyroValue(int highReg, int lowReg) {
        int high = i2cReadByteData(i2cHandle, MPU6050_SLAVE_ADDRESS, highReg);
        int low = i2cReadByteData(i2cHandle, MPU6050_SLAVE_ADDRESS, lowReg);
        int16_t value = (high << 8) | low;

        if (value >= 0x8000) {
            value = -(65535 - value + 1);
        }

        return value / GYRO_SCALE_MODIFIER_250DEG;
    }

    void readGyroscope() {
        for (int i = 0; i < 1000; i++) {
            float gyroX = readGyroValue(GYRO_XOUT0, GYRO_XOUT0 + 1);
            float gyroY = readGyroValue(GYRO_YOUT0, GYRO_YOUT0 + 1);
            float gyroZ = readGyroValue(GYRO_ZOUT0, GYRO_ZOUT0 + 1);

            std::cout << "Gyro X: " << gyroX << " Gyro Y: " << gyroY << " Gyro Z: " << gyroZ << std::endl;

            usleep(10000); 
        }
    }
};

// for the test.cpp
int main() {
    // Create an MPU6050 object to initialize the sensor
    MPU6050 mpu;

    // Read and display gyroscope values
    mpu.readGyroscope();

    return 0;
}
