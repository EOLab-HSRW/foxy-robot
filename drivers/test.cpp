#include <iostream>
#include <unistd.h>
#include "VL53L0X.hpp"

int main(){

    int JETSON_NANO = 0;
    int JETSON_ORIN = 1;

    VL53L0X vl53l0x(JETSON_ORIN, 0);

    VL53L0X::StatusCode status = vl53l0x.init();

    switch (status) {
        case VL53L0X::StatusCode::SUCCESS:
            std::cout << "Everything is good my friend" << std::endl;
            break;
        case VL53L0X::StatusCode::GPIO_INIT_FAILED:
            std::cout << "Failed to init the GPIO library" << std::endl;
            break;
        case VL53L0X::StatusCode::I2C_INIT_FAILED:
            std::cout << "Failed to stablish I2C communication" << std::endl;
            break;
        case VL53L0X::StatusCode::VL53L0X_NOT_FOUND:
            std::cout << "Maybe I'm not taking to a VL53L0X device in this address" << std::endl;
            break;
        case VL53L0X::StatusCode::WRITE_FAILED:
            std::cout << "Something went wrong while writing..." << std::endl;
            break;
        case VL53L0X::StatusCode::READ_FAILED:
            std::cout << "Something went wrong while READING..." << std::endl;
            break;
    }

    int x = 0;

    while (x<1000) {

        std::cout << vl53l0x.readRangeSingleMillimeters() << std::endl;

        usleep(1000);
        x++;
    }

    return 0;
}
