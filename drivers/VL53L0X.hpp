/* Usage example of the JETGPIO library
 * Compile with: g++ -Wall -o jetgpio_i2c_example jetgpio_i2c_example.cpp -ljetgpio
 * Execute with: sudo ./jetgpio_i2c_example
 */
#pragma once
#ifndef VL53L0X_h
#define VL53L0X_h

#include <cstdint>
#include <chrono>
#include <jetgpio.h>


class Timeout {
public:
    Timeout() : timeout_start_ms(0), io_timeout(1000) {} //timeout at 1000 milliseconds

    void start() {
        timeout_start_ms = millis();
    }

    bool hasExpired() const {
        return io_timeout > 0 && (static_cast<uint16_t>(millis() - timeout_start_ms) > io_timeout);
    }

    void setTimeout(uint16_t timeout) {
        io_timeout = timeout;
    }

private:
    uint64_t millis() const {
        // Use a high-resolution clock or appropriate timing function
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    uint64_t timeout_start_ms;
    uint16_t io_timeout;
};


#define VL53L0X_ADDRESS 0x29

class VL53L0X
{
public:
    // register addresses from API vl53l0x_device.h (ordered as listed there)
    enum StatusCode {

        SUCCESS = 0,
        GPIO_INIT_FAILED,       // The GPIO library fail to start
        I2C_INIT_FAILED,
        VL53L0X_NOT_FOUND,      // In case that the device in that I2C address is not a VL53L0X.
        WRITE_FAILED,
        READ_FAILED,
        TIMEOUT,
        UNKNOWN_ERROR
    };

    enum regAddr
    {
      SYSRANGE_START                              = 0x00,

      SYSTEM_THRESH_HIGH                          = 0x0C,
      SYSTEM_THRESH_LOW                           = 0x0E,

      SYSTEM_SEQUENCE_CONFIG                      = 0x01,
      SYSTEM_RANGE_CONFIG                         = 0x09,
      SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

      SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

      GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

      SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

      RESULT_INTERRUPT_STATUS                     = 0x13,
      RESULT_RANGE_STATUS                         = 0x14,

      RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
      RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
      RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
      RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
      RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

      ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

      I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

      MSRC_CONFIG_CONTROL                         = 0x60,

      PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
      PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
      PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
      PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

      FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
      FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
      FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
      FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

      PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
      PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

      PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

      SYSTEM_HISTOGRAM_BIN                        = 0x81,
      HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
      HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

      FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
      CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

      MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

      SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
      IDENTIFICATION_MODEL_ID                     = 0xC0,
      IDENTIFICATION_REVISION_ID                  = 0xC2,

      OSC_CALIBRATE_VAL                           = 0xF8,

      GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

      GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
      DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
      DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
      POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

      VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

      ALGO_PHASECAL_LIM                           = 0x30,
      ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
    };

    // Public members
    uint8_t address;

    // Constructors
    // Check the Jetson GPIO to set the correct I2C handle number and speed
    VL53L0X(unsigned handle = 1, unsigned i2cFlags = 1, uint8_t address = VL53L0X_ADDRESS);
    /**<
     * @brief create VL53L0X object.
     * @param handle I2C handle for communication. 0 are pins 27 (SDA) & 28 (SCL), 1 are pins 3(SDA) & 5(SCL) In Orin the pins are the same but the i2c devices are: 0->i2c-1 & 1->i2c-7
     * @param i2cFlags 0 -> 100 kHz
     * @param i2cFlags 1 -> 400 kHz
     * @param i2cFlags 2 -> 1 MHz
     * @param address 0x29 ToF sensor I2C address.
    */

    StatusCode init();

    StatusCode writeReg(uint8_t reg, uint8_t value);

    int readReg(uint8_t reg);
    int readReg16Bit(uint8_t reg);

    bool performSingleRefCalibration(uint8_t vhv_init_byte);

    uint16_t readRangeSingleMillimeters();

    uint16_t readRangeContinuousMillimeters();

    void setAddress(uint8_t new_addr);


private:
    unsigned handle;
    unsigned i2cFlags;
    int i2c_bus;
    Timeout timer;

};
#endif
