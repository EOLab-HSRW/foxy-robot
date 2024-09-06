# NOTE
# Pinout MPU-6050 communication over I2C via bus(0) of the Jetson
# Install required libraries:
# sudo pip3 install Jetson.GPIO
# sudo apt-get install python-smbus
# The code involves 3 sections; initialize the driver, configure it and reading the acc and gyrsc data


import smbus
import time

# MPU-6050 I2C address
MPU6050_ADDR = 0x68

# MPU-6050 Registers
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43
ACCEL_CONFIG = 0x1C
GYRO_CONFIG = 0x1B

# Initialize I2C - Bus connection of the Jetson Nano
bus = smbus.SMBus(0)  

# Function initializes MPU-6050
def init_mpu6050():
    # This wakes up MPU6050 by writing 0 to PWR_MGMT_1 register
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    # Setting the  accelerometer and gyroscope ranges
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00)  # ±2g
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)   # ±250°/s
    print("MPU6050 Initialized") # Just a print statement

# This function reads raw data from registers
def read_raw_data(addr):
    # Read two bytes of data
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    # Combine the two bytes (MSB first)
    value = (high << 8) | low # an OR operation with the low byte
    # Convert to signed 16-bit integer (two's complement) to account for negative values
    if value > 32768:
        value = value - 65536
    return value

# Function reads accelerometer data
def read_accel():
    accel_x = read_raw_data(ACCEL_XOUT_H)
    accel_y = read_raw_data(ACCEL_XOUT_H + 2)
    accel_z = read_raw_data(ACCEL_XOUT_H + 4)
    return accel_x, accel_y, accel_z

# Function reads gyroscope data
def read_gyro():
    gyro_x = read_raw_data(GYRO_XOUT_H)
    gyro_y = read_raw_data(GYRO_XOUT_H + 2)
    gyro_z = read_raw_data(GYRO_XOUT_H + 4)
    return gyro_x, gyro_y, gyro_z

# Main function
if __name__ == "__main__":
    init_mpu6050()  # Initialize MPU-6050
    
    try:
        while True:
            # Read accelerometer and gyroscope data
            accel_x, accel_y, accel_z = read_accel()
            gyro_x, gyro_y, gyro_z = read_gyro()
            
            # Print the data
            print(f"Accel X: {accel_x}, Y: {accel_y}, Z: {accel_z}")
            print(f"Gyro X: {gyro_x}, Y: {gyro_y}, Z: {gyro_z}")
            
            time.sleep(1)  # Delay just for testing/readability
            
    except KeyboardInterrupt:
        print("Exiting...")
