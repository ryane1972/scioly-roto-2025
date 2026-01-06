from pololu_3pi_2040_robot import robot
import time
import math

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
min_motor_speed = 350
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

moveCount = 0
delayCount = 0

startTime = 0

#TUNING HERE

iteration = 1
"""----------------Forward--------------"""
forwardAccelConst = 20000
forwardDecelConst = 10000
forwardDecelStart = 1200
forwardKp1 = 100
forwardKp2 = 15
forwardKp3 = 25
forwardStopOffset = 3

"""-------------Backwards---------------"""
backwardAccelConst = 10000
backwardDecelConst = 20000
backwardDecelStart = 680
backwardKp1 = 100
backwardKp2 = 15
backwardKp3 = 25
backwardStopOffset = 3

"""-----------Turns----------------"""
turnLeftTarget = 591
turnRightTarget = 591

turnAccelConst = 25000
turnDecelConst = 25000
turnSpeed = 4500
turnDecelStart = 345
turnKp1 = 100
turnKp2 = 50
turnKp3 = 50
leftTurnStopOffset = 0
rightTurnStopOffset = 1
minspeedBoost = 0


def forward(distance, speed):
    global moveCount
    accelConst = forwardAccelConst
    decelConst = forwardDecelConst
    speed = max(min_motor_speed, min(max_motor_speed, speed))
    start_left, start_right = encoders.get_counts()
    target_counts = distance * counts_per_cm

    #accel
    currentSpeed = min_motor_speed
    increment = 0
    while True:
        currentSpeed = float(min_motor_speed + increment*(accelConst/100))
        current_left, current_right = encoders.get_counts()
        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2
        if currentSpeed >= speed or abs(target_counts - traveledCounts) <= forwardDecelStart:
            currentSpeed = speed
            #accelDiff = (current_left - start_left) - (current_right - start_right)
            break
        encoder_error = (current_left - start_left) - (current_right - start_right)
        correction = encoder_error * forwardKp1                                             #KP TUNE HERE
        currentSpeed = max(min_motor_speed, min(max_motor_speed, currentSpeed))
        left_speed = currentSpeed - correction
        right_speed = currentSpeed + correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))

        motors.set_speeds(left_speed, right_speed)
        increment = increment + 1
        time.sleep(0.01)
    #full speed
    while True: 
        current_left, current_right = encoders.get_counts()
        encoder_error = (current_left - start_left) - (current_right - start_right)
        correction = encoder_error * forwardKp2                                             #KP TUNE HERE

        left_speed = speed - correction
        right_speed = speed + correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))

        motors.set_speeds(left_speed, right_speed)

        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2

        # Check if it's time to decelerate
        if abs(target_counts - traveledCounts) <= forwardDecelStart:                            #DECEL LENGTH TUNE HERE
            increment = 0
            break
        time.sleep(0.01)

    #decel
    debugFlag = False
    debugVar = 0
    while True: 
        currentSpeed = float(speed - increment*(decelConst/100))
        current_left, current_right = encoders.get_counts()
        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2
        if traveledCounts >= target_counts - forwardStopOffset:                                  #DISTANCE MOD TUNE HERE
            if debugVar == False:
                debugVar = "b " + str(currentSpeed) + "; " + str(min_motor_speed)
            break
        encoder_error = (current_left - start_left) - (current_right - start_right)
        correction = encoder_error * forwardKp3                                         #KP TUNE HERE
        currentSpeedClamp = max(min_motor_speed, min(max_motor_speed, currentSpeed))
        if currentSpeedClamp == min_motor_speed and debugFlag == False:
            debugFlag = True
            debugVar = "g " + str(traveledCounts) + "; " + str(target_counts-traveledCounts)
        left_speed = currentSpeedClamp - correction
        right_speed = currentSpeedClamp + correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))

        motors.set_speeds(left_speed, right_speed)
        increment = increment + 1
        time.sleep(0.01)

    motors.off()
    time.sleep(0.1)
    moveCount += 1
    print("forwardAccel complete.")
    current_counts_L, current_counts_R = encoders.get_counts()
    display.fill(0)
    display.text("Left: " + str(current_counts_L-start_left), 0, 0, 1)
    display.text("Right: " + str(current_counts_R-start_right), 0, 16, 1)
    display.text(debugVar, 0, 32, 1)
    display.text("Targ: " + str(target_counts), 0, 48, 1)
    display.show()

def backward(distance, speed):
    global moveCount
    accelConst = backwardAccelConst
    decelConst = backwardDecelConst
    speed = max(min_motor_speed, min(max_motor_speed, speed))
    start_left, start_right = encoders.get_counts()
    target_counts = distance * counts_per_cm

    #accel
    currentSpeed = min_motor_speed
    increment = 0
    while True:
        currentSpeed = float(min_motor_speed + increment*(accelConst/100))
        current_left, current_right = encoders.get_counts()
        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2
        if currentSpeed >= speed or abs(target_counts - traveledCounts) <= backwardDecelStart:
            currentSpeed = speed
            #accelDiff = (current_left - start_left) - (current_right - start_right)
            break
        encoder_error = (current_left - start_left) - (current_right - start_right)
        correction = encoder_error * backwardKp1                                             #KP TUNE HERE
        currentSpeed = max(min_motor_speed, min(max_motor_speed, currentSpeed))
        left_speed = currentSpeed + correction
        right_speed = currentSpeed - correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))

        motors.set_speeds(-left_speed, -right_speed)
        increment = increment + 1
        time.sleep(0.01)
    #full speed
    while True: 
        current_left, current_right = encoders.get_counts()
        encoder_error = (current_left - start_left) - (current_right - start_right)
        correction = encoder_error * backwardKp2                                             #KP TUNE HERE

        left_speed = speed + correction
        right_speed = speed - correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))

        motors.set_speeds(-left_speed, -right_speed)

        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2

        # Check if it's time to decelerate
        if abs(target_counts - traveledCounts) <= backwardDecelStart:                            #DECEL LENGTH TUNE HERE
            increment = 0
            break
        time.sleep(0.01)

    #decel
    debugFlag = False
    debugVar = 0
    while True: 
        currentSpeed = float(speed - increment*(decelConst/100))
        current_left, current_right = encoders.get_counts()
        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2
        if traveledCounts >= target_counts - backwardStopOffset:                                  #DISTANCE MOD TUNE HERE
            if debugVar == False:
                debugVar = "b " + str(currentSpeed) + "; " + str(min_motor_speed)
            break
        encoder_error = (current_left - start_left) - (current_right - start_right)
        correction = encoder_error * backwardKp3                                         #KP TUNE HERE
        currentSpeedClamp = max(min_motor_speed, min(max_motor_speed, currentSpeed))
        if currentSpeedClamp == min_motor_speed and debugFlag == False:
            debugFlag = True
            debugVar = "g " + str(traveledCounts) + "; " + str(target_counts-traveledCounts)
        left_speed = currentSpeedClamp + correction
        right_speed = currentSpeedClamp - correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))

        motors.set_speeds(-left_speed, -right_speed)
        increment = increment + 1
        time.sleep(0.01)

    motors.off()
    time.sleep(0.1)
    moveCount += 1
    print("forwardAccel complete.")
    current_counts_L, current_counts_R = encoders.get_counts()
    display.fill(0)
    display.text("Left: " + str(current_counts_L-start_left), 0, 0, 1)
    display.text("Right: " + str(current_counts_R-start_right), 0, 16, 1)
    display.text(debugVar, 0, 32, 1)
    display.text("Targ: " + str(target_counts), 0, 48, 1)
    display.show()

def left():
    global moveCount
    accelConst = turnAccelConst
    decelConst = turnDecelConst
    speed = turnSpeed
    start_left, start_right = encoders.get_counts()
    target_counts = turnLeftTarget

    #accel
    currentSpeed = min_motor_speed
    increment = 0
    while True:
        currentSpeed = float(min_motor_speed + increment*(accelConst/100))
        current_left, current_right = encoders.get_counts()
        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2
        if currentSpeed >= speed or abs(target_counts - traveledCounts) <= turnDecelStart:
            currentSpeed = speed
            break
        encoder_error = (current_left - start_left) + (current_right - start_right)
        correction = encoder_error * turnKp1 #KP TUNE HERE
        currentSpeed = max(min_motor_speed, min(max_motor_speed, currentSpeed))
        left_speed = currentSpeed + correction
        right_speed = currentSpeed - correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))
        left_speed = -left_speed

        motors.set_speeds(left_speed, right_speed)
        increment = increment + 1
        time.sleep(0.01)

    #full speed
    while True: 
        current_left, current_right = encoders.get_counts()
        encoder_error = (current_left - start_left) + (current_right - start_right)
        correction = encoder_error * turnKp2 #KP TUNE HERE

        left_speed = speed + correction
        right_speed = speed - correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))
        left_speed = -left_speed

        motors.set_speeds(left_speed, right_speed)

        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2

        # Check if it's time to decelerate
        if abs(target_counts - traveledCounts) <= turnDecelStart: #DECEL LENGTH TUNE HERE
            increment = 0
            break
        time.sleep(0.01)

    #decel
    debugFlag = False
    debugVar = 0
    while True: 
        currentSpeed = float(speed - increment*(decelConst/100))
        current_left, current_right = encoders.get_counts()
        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2
        if traveledCounts >= target_counts - leftTurnStopOffset: #DISTANCE MOD TUNE HERE
            if debugVar == False:
                debugVar = "b " + str(currentSpeed) + "; " + str(min_motor_speed)
            break
        encoder_error = (current_left - start_left) + (current_right - start_right)
        correction = encoder_error * turnKp3 #KP TUNE HERE
        currentSpeedClamp = max(min_motor_speed, min(max_motor_speed, currentSpeed))
        if currentSpeedClamp == min_motor_speed and debugFlag == False:
            debugFlag = True
            debugVar = "g " + str(traveledCounts) + "; " + str(target_counts-traveledCounts)
        left_speed = currentSpeedClamp + correction
        right_speed = currentSpeedClamp - correction
        left_speed = max(min_motor_speed+minspeedBoost, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed+minspeedBoost, min(max_motor_speed, right_speed))
        left_speed = -left_speed

        motors.set_speeds(left_speed, right_speed)
        increment = increment + 1
        time.sleep(0.01)

    motors.off()
    moveCount += 1
    print("forwardAccel complete.")
    current_counts_L, current_counts_R = encoders.get_counts()
    display.fill(0)
    display.text("Left: " + str(current_counts_L-start_left), 0, 0, 1)
    display.text("Right: " + str(current_counts_R-start_right), 0, 16, 1)    
    display.text(debugVar, 0, 32, 1)
    display.text("Targ: " + str(target_counts), 0, 48, 1)
    display.show()

def right():
    global moveCount
    accelConst = turnAccelConst
    decelConst = turnDecelConst
    speed = turnSpeed
    start_left, start_right = encoders.get_counts()
    target_counts = turnRightTarget

    #accel
    currentSpeed = min_motor_speed
    increment = 0
    while True:
        currentSpeed = float(min_motor_speed + increment*(accelConst/100))
        current_left, current_right = encoders.get_counts()
        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2
        if currentSpeed >= speed or abs(target_counts - traveledCounts) <= turnDecelStart+rightTurnStopOffset:
            currentSpeed = speed
            break
        encoder_error = (current_left - start_left) + (current_right - start_right)
        correction = encoder_error * turnKp1 #KP TUNE HERE
        currentSpeed = max(min_motor_speed, min(max_motor_speed, currentSpeed))
        left_speed = currentSpeed - correction
        right_speed = currentSpeed + correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))
        right_speed = -right_speed

        motors.set_speeds(left_speed, right_speed)
        increment = increment + 1
        time.sleep(0.01)

    #full speed
    while True: 
        current_left, current_right = encoders.get_counts()
        encoder_error = (current_left - start_left) + (current_right - start_right)
        correction = encoder_error * turnKp2 #KP TUNE HERE

        left_speed = speed - correction
        right_speed = speed + correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))
        right_speed = -right_speed

        motors.set_speeds(left_speed, right_speed)

        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2

        # Check if it's time to decelerate
        if abs(target_counts - traveledCounts) <= turnDecelStart+rightTurnStopOffset: #DECEL LENGTH TUNE HERE
            increment = 0
            break
        time.sleep(0.01)

    #decel
    debugFlag = False
    debugVar = 0
    while True: 
        currentSpeed = float(speed - increment*(decelConst/100))
        current_left, current_right = encoders.get_counts()
        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2
        if traveledCounts >= target_counts - rightTurnStopOffset: #DISTANCE MOD TUNE HERE
            if debugVar == False:
                debugVar = "b " + str(currentSpeed) + "; " + str(min_motor_speed)
            break
        encoder_error = (current_left - start_left) + (current_right - start_right)
        correction = encoder_error * turnKp3 #KP TUNE HERE
        currentSpeedClamp = max(min_motor_speed, min(max_motor_speed, currentSpeed))
        if currentSpeedClamp == min_motor_speed and debugFlag == False:
            debugFlag = True
            debugVar = "g " + str(traveledCounts) + "; " + str(target_counts-traveledCounts)
        left_speed = currentSpeedClamp - correction
        right_speed = currentSpeedClamp + correction
        left_speed = max(min_motor_speed+minspeedBoost, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed+minspeedBoost, min(max_motor_speed, right_speed))
        right_speed = -right_speed

        motors.set_speeds(left_speed, right_speed)
        increment = increment + 1
        time.sleep(0.01)

    motors.off()
    moveCount += 1
    print("forwardAccel complete.")
    current_counts_L, current_counts_R = encoders.get_counts()
    display.fill(0)
    display.text("Left: " + str(current_counts_L-start_left), 0, 0, 1)
    display.text("Right: " + str(current_counts_R-start_right), 0, 16, 1)    
    display.text(debugVar, 0, 32, 1)
    display.text("Targ: " + str(target_counts), 0, 48, 1)
    display.show()

def forwardAccelDiagnogstic(distance, speed):
    global moveCount
    accelConst = 20000
    decelConst = 10000
    speed = max(min_motor_speed, min(max_motor_speed, speed))
    start_left, start_right = encoders.get_counts()
    target_counts = distance * counts_per_cm

    #accel
    currentSpeed = min_motor_speed
    increment = 0
    while True:
        currentSpeed = float(min_motor_speed + increment*(accelConst/100))
        current_left, current_right = encoders.get_counts()
        if currentSpeed >= speed:
            currentSpeed = speed
            accelDiff = (current_left - start_left) - (current_right - start_right)
            break
        encoder_error = (current_left - start_left) - (current_right - start_right)
        correction = encoder_error * 100                                             #KP TUNE HERE
        currentSpeed = max(min_motor_speed, min(max_motor_speed, currentSpeed))
        left_speed = currentSpeed - correction
        right_speed = currentSpeed + correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))

        motors.set_speeds(left_speed, right_speed)
        increment = increment + 1
        time.sleep(0.01)

    #full speed
    while True: 
        current_left, current_right = encoders.get_counts()
        encoder_error = (current_left - start_left) - (current_right - start_right)
        correction = encoder_error * 15                                              #KP TUNE HERE

        left_speed = speed - correction
        right_speed = speed + correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))

        motors.set_speeds(left_speed, right_speed)

        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2

        # Check if it's time to decelerate
        if abs(target_counts - traveledCounts) <= 1100:                            #DECEL LENGTH TUNE HERE
            increment = 0
            constDiff = (current_left - start_left) - (current_right - start_right)
            break
        time.sleep(0.01)

    #decel
    debugFlag = False
    debugVar = 0
    while True: 
        currentSpeed = float(speed - increment*(decelConst/100))
        current_left, current_right = encoders.get_counts()
        traveledCounts = (abs(current_left - start_left) + abs(current_right - start_right)) / 2
        if traveledCounts >= target_counts - 0:                                  #DISTANCE MOD TUNE HERE
            if debugVar == False:
                debugVar = "b " + str(currentSpeed) + "; " + str(min_motor_speed)
            break
        encoder_error = (current_left - start_left) - (current_right - start_right)
        correction = encoder_error * 25                                          #KP TUNE HERE
        currentSpeedClamp = max(min_motor_speed, min(max_motor_speed, currentSpeed))
        if currentSpeedClamp == min_motor_speed and debugFlag == False:
            debugFlag = True
            debugVar = "g " + str(traveledCounts) + "; " + str(target_counts-traveledCounts)
        left_speed = currentSpeedClamp - correction
        right_speed = currentSpeedClamp + correction
        left_speed = max(min_motor_speed, min(max_motor_speed, left_speed))
        right_speed = max(min_motor_speed, min(max_motor_speed, right_speed))

        motors.set_speeds(left_speed, right_speed)
        increment = increment + 1
        time.sleep(0.01)

    motors.off()
    time.sleep(0.1)
    moveCount += 1
    print("forwardAccel complete.")
    current_counts_L, current_counts_R = encoders.get_counts()
    while True:
        display.fill(0)
        display.text("Left: " + str(current_counts_L-start_left), 0, 0, 1)
        display.text("Right: " + str(current_counts_R-start_right), 0, 16, 1)
        display.text(debugVar, 0, 32, 1)
        display.text("Targ: " + str(target_counts), 0, 48, 1)
        display.show()
        time.sleep(5)
        display.fill(0)
        display.text("Acc df: " + str(accelDiff), 0, 0, 1)
        display.text("Cst df: " + str(constDiff), 0, 16, 1)
        display.show()
        time.sleep(5)

def prerun():
    global startTime

    # Display message
    display.fill(0)
    display.text("States", 0, 0, 1)
    display.text("Run: " + str(iteration), 0, 16, 1)
    display.text("Press B to run", 0, 32, 1)
    display.text("Please work", 0, 48, 1)
    display.show()

    # Wait until button B is pressed
    while not button_b.check():
        pass  # Button B has to be pressed

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
    startTime = time.ticks_ms()

delayTime = 0.523256
targetTime = 70

def delay():
    global delayCount
    delayCount += 1
    time.sleep(delayTime)

prerun()

forward(60.0, 5750)
delay()
backward(30, 5750)
delay()
left()
delay()
forward(150.0, 5750)
delay()
right()
delay()
forward(50.0, 5750)
delay()
left()
delay()
forward(50.0, 5750)
delay()
left()
delay()
forward(30.0, 5750)
delay()
backward(30.0, 5750)
delay()
left()
delay()
forward(150.0, 5750)
delay()
left()
delay()
forward(100.0, 5750)
delay()
left()
delay()
forward(30.0, 5750)
delay()
backward(30.0, 5750)
delay()
left()
delay()
forward(50.0, 5750)
delay()
left()
delay()
forward(50.0, 5750)
delay()
left()
delay()
forward(30.0, 5750)
delay()
backward(30, 5750)
delay()
left()
delay()
forward(50.0, 5750)
delay()
left()
delay()
forward(50.0, 5750)
delay()
right()
delay()
forward(150.0, 5750)
delay()
right()
delay()
forward(107.75, 5750)
delay()
right()
delay()
forward(42.0, 5750)

endTime = time.ticks_ms()
runTime = (endTime - startTime)/1000
timeDelay = delayTime*delayCount
runTimeRaw = runTime - delayTime
neededDelay = targetTime - runTimeRaw
recommendedDelay = neededDelay/delayCount
display.fill(0)
display.text(str(runTime), 0, 0, 1)
display.text(str(delayTime), 0, 16, 1)
display.text(str(timeDelay), 0, 32, 1)
display.text(str(recommendedDelay + delayTime), 0, 48, 1)
display.show()