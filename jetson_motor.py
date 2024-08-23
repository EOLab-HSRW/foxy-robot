# using the motor.py
from motor import Motor, MotorDirection, MotorDirectionControl
from Adafruit_PWM_Servo_Driver import PWM

import time

# Initialize the PWM device using the I2C address of the HAT
pwm = PWM(0x40)  # The I2C address of the HUT

# Set PWM frequency to 1600 Hz
pwm.setPWMFreq(1600)

# Define motor pins and control method to use
motor1_pins = MotorPins(in1=10, in2=9, pwm=8, control=MotorDirectionControl.PWM)
motor2_pins = MotorPins(in1=33, in2=31, pwm=13, control=MotorDirectionControl.GPIO)



# Instantiate Motor objects
motor1 = Motor(name="Motor1", pwm=pwm, in1_pin=motor1_pins.in1, in2_pin=motor1_pins.in2, pwm_pin=motor1_pins.pwm, control=motor1_pins.control)
motor2 = Motor(name="Motor2", pwm=pwm, in1_pin=motor2_pins.in1, in2_pin=motor2_pins.in2, pwm_pin=motor2_pins.pwm, control=motor2_pins.control)

# Control Motor1:using PWM control
motor1.set(direction=MotorDirection.FORWARD, speed=128)  # This sets motor1 to move forward 

# Control Motor2:using GPIO control
motor2.set(direction=MotorDirection.BACKWARD, speed=200)  # This sets motor2 to move backward

# Introduce a time to stop motors after some time
time.sleep(2)

# Stop Motor1 
motor1.set(direction=MotorDirection.RELEASE, speed=0)

# Stop Motor2
motor2.set(direction=MotorDirection.RELEASE, speed=0)

