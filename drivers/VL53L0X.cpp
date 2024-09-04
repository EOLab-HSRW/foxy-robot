/* Usage example of the JETGPIO library
 * Compile with: g++ -Wall -o jetgpio_i2c_example jetgpio_i2c_example.cpp -ljetgpio
 * Execute with: sudo ./jetgpio_i2c_example
 */
#include <cstdint>
#include <iostream>
#include <jetgpio.h>
#include "VL53L0X.hpp"



VL53L0X::VL53L0X(unsigned handle, unsigned i2cFlags, uint8_t address)
: address(address), handle(handle), i2cFlags(i2cFlags) {
}

VL53L0X::StatusCode VL53L0X::init() {

    Timeout timer;

    StatusCode status;

    int gpio_init = gpioInitialise();


    if (gpio_init < 0) {
        return StatusCode::GPIO_INIT_FAILED;
    }

    i2c_bus = i2cOpen(handle, i2cFlags);


    if (i2c_bus < 0) {
        return StatusCode::I2C_INIT_FAILED;
    }

    // verify that we are talking with a VL53L0X sensor in the I2C bus
    // Check Table 5 from ST Datasheet
    if (readReg(IDENTIFICATION_MODEL_ID) != 0xEE) {
        return StatusCode::VL53L0X_NOT_FOUND;
    }

    // set the device to I2C Standard Mode
    // from the ST API
    // https://github.com/adafruit/Adafruit_VL53L0X/blob/23dc309c56ae7b3031b296380b0663c5dd5edb68/src/core/src/vl53l0x_api.cpp#L362-L364
    status = writeReg(0x88, 0x00);
    if (status != StatusCode::SUCCESS) return StatusCode::WRITE_FAILED;

    const uint8_t data_init[][2] = {
        {0x80, 0x81}, {0xFF, 0x01}, {0x00, 0x00},
        {0x91, 0x00}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}
    };


    // data initialization from ST API
    // https://github.com/adafruit/Adafruit_VL53L0X/blob/23dc309c56ae7b3031b296380b0663c5dd5edb68/src/core/src/vl53l0x_api.cpp#L414-L421
    for (const auto& data : data_init) {
        status = writeReg(data[0], data[1]);
        if (status != StatusCode::SUCCESS) return StatusCode::WRITE_FAILED;
    }

    // -- VL53L0X_load_tuning_settings() begin
    // DefaultTuningSettings from vl53l0x_tuning.h

    // Load default tuning settings
    const uint8_t settings[][2] = {
        {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00},
        {0x10, 0x00}, {0x11, 0x00}, {0x24, 0x01}, {0x25, 0xFF},

        {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
        {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00},

        {0x31, 0x04}, {0x32, 0x03}, {0x40, 0x83}, {0x46, 0x25},
        {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},

        {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00},
        {0x62, 0x00}, {0x64, 0x00}, {0x65, 0x00}, {0x66, 0xA0},

        {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
        {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00},

        {0x78, 0x21}, {0xFF, 0x01}, {0x23, 0x34}, {0x42, 0x00},
        {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},

        {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00},
        {0x34, 0x03}, {0x35, 0x44}, {0xFF, 0x01}, {0x31, 0x04},

        {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
        {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28},

        {0x67, 0x00}, {0x70, 0x04}, {0x71, 0x01}, {0x72, 0xFE},
        {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},

        {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01},
        {0x8E, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00}
    };

    for (const auto& setting : settings) {
        status = writeReg(setting[0], setting[1]);
        if (status != StatusCode::SUCCESS) return StatusCode::WRITE_FAILED;
    }

    // -- VL53L0X_load_tuning_settings() end

    // // "Set interrupt config to new sample ready"
    // -- VL53L0X_SetGpioConfig() begin

    writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    // -- VL53L0X_SetGpioConfig() end
    //
    /// "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // -- VL53L0X_SetSequenceStepEnable() end


    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin

    writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(0x40)) { return UNKNOWN_ERROR; }

    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(0x00)) { return UNKNOWN_ERROR; }

    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // VL53L0X_PerformRefCalibration() end

    return SUCCESS;
}


int VL53L0X::readReg(uint8_t reg) {
    int value = i2cReadByteData(i2c_bus, address, reg);
    return value;
}

int VL53L0X::readReg16Bit(uint8_t reg) {
    int value = i2cReadWordData(i2c_bus, address, reg);
    return value;
}

VL53L0X::StatusCode VL53L0X::writeReg(uint8_t reg, uint8_t data) {
    int status = i2cWriteByteData(i2c_bus, address, reg, data );

    if (status < 0) {
        std::cout << "WRITE_FAILED, register " << reg << std::endl;
        return StatusCode::WRITE_FAILED;
    }

    return SUCCESS;
}

// based on VL53L0X_perform_single_ref_calibration()
bool VL53L0X::performSingleRefCalibration(uint8_t vhv_init_byte)
{
    writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

    timer.start();
    while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (timer.hasExpired()) {
            return false; 
        }
    }

    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    writeReg(SYSRANGE_START, 0x00);

    return true;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t VL53L0X::readRangeSingleMillimeters()
{
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    // writeReg(0x91, stop_variable);
    writeReg(0x91, 0x00);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    writeReg(SYSRANGE_START, 0x01);

    // "Wait until start bit has been cleared"
    timer.start();
    while (readReg(SYSRANGE_START) & 0x01)
    {
        if (timer.hasExpired())
        {
            // did_timeout = true;
            return 65535;
        }
    }

    return readRangeContinuousMillimeters();
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t VL53L0X::readRangeContinuousMillimeters()
{
    timer.start();
    while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (timer.hasExpired())
        {
            //did_timeout = true;
            return 65535;
        }
    }

    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    uint16_t range = readReg16Bit(RESULT_RANGE_STATUS + 10);

    writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range;
}
