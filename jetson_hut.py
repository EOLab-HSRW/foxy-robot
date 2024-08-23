from hat import HATv1, HATv2, HATv3

# Initialize the HAT
hat = HATv1(address=0x40) # Define the hut address to use(0x40)
hat = HATv3(HATv2)

# Use the hat to retrieve a motor 
motor = hat.get_motor(1, 'Motor1') # give a name to this motor 

# Set motor speed (128)
motor.set_speed(128)

# Set motor direction
motor.forward()  # This moves the motor forward


# Stop the motor
motor.stop()
