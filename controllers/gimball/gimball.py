from controller import Robot, Keyboard

# Constants for PID control
KP = 0.5  # Proportional gain
KI = 0.1  # Integral gain
KD = 0.2  # Derivative gain

# Variables for PID control
last_error = [0, 0]   # Previous error values for derivative term
integral = [0, 0]     # Accumulated error for integral term

# Target angles for stabilization (in radians)
target_angles = [0, 0]  # Desired gimbal angles (roll, pitch)

# Function to calculate control signal
def calculate_control_signal(current_angles, delta_time):
    # Calculate error for each axis
    errors = [target - current for target, current in zip(target_angles, current_angles)]

    # Calculate proportional term
    proportional = [KP * error for error in errors]

    # Calculate integral term
    integral[0] += errors[0] * delta_time
    integral[1] += errors[1] * delta_time
    integral = [KI * integral_val for integral_val in integral]

    # Calculate derivative term
    derivative = [(error - last) / delta_time for error, last in zip(errors, last_error)]
    derivative = [KD * derivative_val for derivative_val in derivative]

    # Calculate control signal
    control_signal = [p + i + d for p, i, d in zip(proportional, integral, derivative)]

    # Update previous error values
    last_error[0], last_error[1] = errors[0], errors[1]

    return control_signal

# Create a Webots robot instance
robot = Robot()

# Get devices and enable sensors
gimbal_roll_sensor = robot.getDevice("roll")
gimbal_pitch_sensor = robot.getDevice("pitch")
gimbal_roll_sensor.enable(16)  # Enable position sensor with 16ms refresh rate
gimbal_pitch_sensor.enable(16) # Enable position sensor with 16ms refresh rate

gimbal_roll_motor = robot.getDevice("motor3")
gimbal_pitch_motor = robot.getDevice("motor4")
gimbal_roll_motor.setPosition(float('inf'))    # Enable position control
gimbal_pitch_motor.setPosition(float('inf'))   # Enable position control

# Enable keyboard for teleoperation
keyboard = Keyboard()
keyboard.enable(16)  # Enable keyboard input with 16ms refresh rate

# Time step for simulation
time_step = int(robot.getBasicTimeStep())

# Main control loop
while robot.step(time_step) != -1:
    # Get current gimbal angles from sensors (in radians)
    current_angles = [gimbal_roll_sensor.getValue(), gimbal_pitch_sensor.getValue()]

    # Calculate control signal
    control_signal = calculate_control_signal(current_angles, time_step / 1000.0)  # Convert time_step to seconds

    # Apply control signal to adjust gimbal angles
    gimbal_roll_motor.setVelocity(control_signal[0])
    gimbal_pitch_motor.setVelocity(control_signal[1])
    
    key = keyboard.getKey()
    if key == Keyboard.UP:  # Pitch up
        target_angles[1] += 0.1
    elif key == Keyboard.DOWN:  # Pitch down
        target_angles[1] -= 0.1
    elif key == Keyboard.LEFT:  # Roll left
        target_angles[0] -= 0.1
    elif key == Keyboard.RIGHT:  # Roll right
        target_angles[0] += 0.1
    
