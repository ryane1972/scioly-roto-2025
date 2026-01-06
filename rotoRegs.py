from pololu_3pi_2040_robot import robot
import time

angle_to_turn = 90
turn_speed_factor = 1.0  # Adjust between 0.1 - 1.0 for speed

motors = robot.Motors()
encoders = robot.Encoders()
button_a = robot.ButtonA()
button_b = robot.ButtonB()
button_c = robot.ButtonC()
display = robot.Display()
yellow_led = robot.YellowLED()
imu = robot.IMU()

imu.reset()
imu.enable_default()

# Constants
max_motor_speed = 6000
min_motor_speed = 1500
max_speed_mps = 0.6
min_speed_mps = 0.05

wheel_diameter = 0.033
counts_per_revolution = 900
wheel_circumference = 3.14159 * wheel_diameter
counts_per_meter = counts_per_revolution / wheel_circumference
counts_per_cm = counts_per_meter / 100

# Gyro variables
drive_motors = False
last_time_gyro_reading = None
target_angle = 0.0
robot_angle = 0.0
last_time_far_from_target = None

display.fill(0)
display.text("Press B", 0, 0, 1)
display.text("To Calibrate", 0, 16, 1)
display.show()

while not button_b.check():
    pass  # Button B has to be pressed

display.fill(0)
display.text("Hands Off", 0, 0, 1)
display.show()

time.sleep_ms(2000)

display.fill(0)
display.text("Calibrating...", 0, 0, 1)
display.show()
time.sleep_ms(500)
calibration_start = time.ticks_ms()
stationary_gz = 0.0
reading_count = 0
while time.ticks_diff(time.ticks_ms(), calibration_start) < 1000:
    if imu.gyro.data_ready():
        imu.gyro.read()
        stationary_gz += imu.gyro.last_reading_dps[2]
        reading_count += 1
stationary_gz /= reading_count

# Display message
display.fill(0)
display.text("Press C", 0, 0, 1)
display.text("To Run", 0, 16, 1)
display.show()

def turn(angle, speed_factor=turn_speed_factor):
    """Turns the robot by a given angle with adjustable speed factor."""
    global target_angle, drive_motors, last_time_gyro_reading, last_time_far_from_target
    target_angle = robot_angle + angle
    drive_motors = True
    last_time_gyro_reading = time.ticks_us()
    last_time_far_from_target = time.ticks_ms()
    
    while drive_motors:
        update_angle()
        control_motors(speed_factor)


#OVER HEREEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE0


def left():
    turn(angle_to_turn + .2)

def right():
    turn(-angle_to_turn + .2)

def update_angle():
    global robot_angle, last_time_gyro_reading
    if imu.gyro.data_ready():
        imu.gyro.read()
        turn_rate = imu.gyro.last_reading_dps[2] - stationary_gz
        now = time.ticks_us()
        if last_time_gyro_reading:
            dt = time.ticks_diff(now, last_time_gyro_reading)
            robot_angle += turn_rate * dt / 1000000
        last_time_gyro_reading = now

def control_motors(speed_factor):
    """Controls motor speed using a speed factor for fine-tuned turning."""
    global drive_motors, last_time_far_from_target
    base_turn_speed = 350 * speed_factor
    base_gyro_correction = 7 * speed_factor

    turn_speed = (target_angle - robot_angle) * base_turn_speed - (imu.gyro.last_reading_dps[2] - stationary_gz) * base_gyro_correction
    turn_speed = max(min(turn_speed, max_motor_speed), -max_motor_speed)
    
    motors.set_speeds(-turn_speed, turn_speed)
    
    far_from_target = abs(robot_angle - target_angle) > 3
    if far_from_target:
        last_time_far_from_target = time.ticks_ms()
    elif time.ticks_diff(time.ticks_ms(), last_time_far_from_target) > 250:
        drive_motors = False
        motors.off()

def forward(distance_cm, speed_mps):
    """Moves forward a specified distance (cm) at a given speed (m/s), correcting drift using encoder feedback."""
    global target_angle
    
    # Get initial encoder counts
    initial_counts_L, initial_counts_R = encoders.get_counts()
    target_counts = distance_cm * counts_per_cm

    # Constrain speed to safe limits
    speed_mps = max(min(speed_mps, max_speed_mps), min_speed_mps)
    base_motor_speed = int((speed_mps - min_speed_mps) / (max_speed_mps - min_speed_mps) * (max_motor_speed - min_motor_speed) + min_motor_speed)

    print(f"Moving forward {distance_cm} cm at {speed_mps} m/s (Motor Speed: {base_motor_speed})")

    # Start both motors at the same speed
    motors.set_speeds(base_motor_speed, base_motor_speed)

    while True:
        # Get current encoder counts
        current_counts_L, current_counts_R = encoders.get_counts()
        
        # Check if we have reached the target distance
        if abs(current_counts_L - initial_counts_L) >= target_counts or abs(current_counts_R - initial_counts_R) >= target_counts:
            break  # Stop moving once the target is reached

        # Calculate encoder difference
        encoder_error = (current_counts_L - initial_counts_L) - (current_counts_R - initial_counts_R)

        # Apply correction: Adjust speeds to reduce encoder error
        correction = encoder_error * 5  # Adjust gain as needed
        left_speed = base_motor_speed - correction
        right_speed = base_motor_speed + correction

        # Ensure speeds stay within motor limits
        left_speed = max(min(left_speed, max_motor_speed), min_motor_speed)
        right_speed = max(min(right_speed, max_motor_speed), min_motor_speed)

        # Set adjusted motor speeds+
        motors.set_speeds(left_speed, right_speed)

        # Small delay for stability
        time.sleep(0.05)

    # Stop motors once target distance is reached
    motors.off()
    print("Movement complete.")

# Wait until button C is pressed
while not button_c.check():
    pass  # Button C has to be pressed

display.fill(0)
display.show()
time.sleep(1)
display.fill(0)
display.text("3", 50, 32, 30)
display.show()
time.sleep(1)
display.fill(0)
display.text("2", 50, 32, 30)
display.show()
display.fill(0)
time.sleep(1)
display.text("1", 50, 32, 30)
display.show()
display.fill(0)
time.sleep(1)
display.fill(0)
display.text("I hate", 0, 16, 1)
display.text("robot", 0, 32, 1)
display.text("tour <3", 0, 48, 1)
display.show()

#MOVE TIMES:
#First: 1.5
#50: 2.15
#100: 4.3
#150: 6.45
#200: 8.6
#Last: 2
#Turn: 0.2
delayTime = 0.34

forward(30, 0.3)  # First move 1.5s
time.sleep(delayTime)
forward(100, 0.3)
time.sleep(delayTime)
left()
time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)
time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)
time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(100, 0.3)
time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
right()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
left()
time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
right()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
right()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
left()
time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
right()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
right()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
left()
time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)

time.sleep(delayTime)
left()
time.sleep(delayTime)
forward(50, 0.3)
time.sleep(delayTime)
forward(47.5, 0.3)  # Move forward 25 cm at 0.05 m/s