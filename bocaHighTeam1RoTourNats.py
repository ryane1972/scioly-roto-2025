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
moveTimes = []
calculatedTimes = []

direction = 0
xError = 0
yError = 0
diagError = 0
rotationalError = 0
leftOffset = 0
rightOffset = 0
rotationalOffset = 0
lastSplit = "left"


#TUNING HERE

airRun = True
iteration = 0
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
turnLeftTarget = 593
turnRightTarget = 593

turnAccelConst = 25000
turnDecelConst = 25000
turnSpeed = 4500
turnDecelStart = 350
turnKp1 = 100
turnKp2 = 50
turnKp3 = 100
leftTurnStopOffset = 0
rightTurnStopOffset = 0
minspeedBoost = 0
reversePower = 250
reverseTime = 0.05

def setDirection(input):
    global direction
    direction += input 
    if (direction > 360):
        direction -= 360
    elif (direction < 0):
        direction += 360

def forward(distance, speed):
    global moveCount
    global moveTimes
    global rotationalError
    accelConst = forwardAccelConst
    decelConst = forwardDecelConst
    speed = max(min_motor_speed, min(max_motor_speed, speed))
    start_left, start_right = encoders.get_counts()
    target_counts = distance * counts_per_cm
    moveStartTime = time.ticks_ms()

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
    time.sleep(0.05)
    moveCount += 1
    moveEndTime = time.ticks_ms()
    moveTime = moveEndTime-moveStartTime
    moveTimes.append(moveTime)
    print("forwardAccel complete.")
    current_counts_L, current_counts_R = encoders.get_counts()
    rotationalError += current_left - start_left - current_right + start_right
    display.fill(0)
    display.text("Left: " + str(current_counts_L-start_left), 0, 0, 1)
    display.text("Right: " + str(current_counts_R-start_right), 0, 16, 1)
    display.text(debugVar, 0, 32, 1)
    display.text("Targ: " + str(target_counts), 0, 48, 1)
    display.show()

def backward(distance, speed):
    global moveCount
    global moveTimes
    global rotationalError
    accelConst = backwardAccelConst
    decelConst = backwardDecelConst
    speed = max(min_motor_speed, min(max_motor_speed, speed))
    start_left, start_right = encoders.get_counts()
    target_counts = distance * counts_per_cm
    moveStartTime = time.ticks_ms()

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
    time.sleep(0.05)
    moveCount += 1
    moveEndTime = time.ticks_ms()
    moveTime = moveEndTime-moveStartTime
    moveTimes.append(moveTime)
    print("forwardAccel complete.")
    current_counts_L, current_counts_R = encoders.get_counts()
    rotationalError += current_left - start_left - current_right + start_right
    display.fill(0)
    display.text("Left: " + str(current_counts_L-start_left), 0, 0, 1)
    display.text("Right: " + str(current_counts_R-start_right), 0, 16, 1)
    display.text(debugVar, 0, 32, 1)
    display.text("Targ: " + str(target_counts), 0, 48, 1)
    display.show()

def left(deg = 90):
    global moveCount
    global moveTimes
    global direction
    global rotationalError
    rotSplit()
    accelConst = turnAccelConst
    decelConst = turnDecelConst
    speed = turnSpeed
    start_left, start_right = encoders.get_counts()
    target_counts = turnLeftTarget*(deg/90) + rotationalOffset
    moveStartTime = time.ticks_ms()
    if airRun:
        display.fill(0)
        display.text("LEFT", 0, 0, 1)
        display.show()

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

    motors.set_speeds(reversePower, -reversePower)
    time.sleep(reverseTime)
    motors.off()
    time.sleep(0.05)
    moveCount += 1
    setDirection(-deg)
    
    moveEndTime = time.ticks_ms()
    moveTime = moveEndTime-moveStartTime
    moveTimes.append(moveTime)
    print("forwardAccel complete.")
    current_counts_L, current_counts_R = encoders.get_counts()
    rotationalError += (target_counts + current_counts_L - start_left) + (target_counts - current_counts_R + start_right)
    display.fill(0)
    display.text("Left: " + str(current_counts_L-start_left), 0, 0, 1)
    display.text("Right: " + str(current_counts_R-start_right), 0, 16, 1)    
    display.text(debugVar, 0, 32, 1)
    display.text("Err: " + str(rotationalError), 0, 48, 1)
    display.show()

def right(deg = 90):
    global moveCount
    global moveTimes
    global direction
    global rotationalError
    rotSplit()
    accelConst = turnAccelConst
    decelConst = turnDecelConst
    speed = turnSpeed
    start_left, start_right = encoders.get_counts()
    target_counts = turnRightTarget*(deg/90) - rotationalOffset
    moveStartTime = time.ticks_ms()
    if airRun:
        display.fill(0)
        display.text("RIGHT", 0, 0, 1)
        display.show()

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

    motors.set_speeds(-reversePower, reversePower)
    time.sleep(reverseTime)
    motors.off()
    time.sleep(0.05)
    moveCount += 1
    setDirection(deg)
    moveEndTime = time.ticks_ms()
    moveTime = moveEndTime-moveStartTime
    moveTimes.append(moveTime)
    print("forwardAccel complete.")
    current_counts_L, current_counts_R = encoders.get_counts()
    rotationalError += (-target_counts + current_counts_L - start_left) + (-target_counts - current_counts_R + start_right)
    display.fill(0)
    display.text("Left: " + str(current_counts_L-start_left), 0, 0, 1)
    display.text("Right: " + str(current_counts_R-start_right), 0, 16, 1)    
    display.text(debugVar, 0, 32, 1)
    display.text("Err: " + str(rotationalError), 0, 48, 1)
    display.show()

def splitRotationalError():
    global rotationalError
    global leftOffset
    global rightOffset
    if rotationalError%2 != 0:
        if rotationalError > 0:
            rotationalError -= 1
            rotationalError /= 2
            leftOffset = rotationalError
            rightOffset = rotationalError
            if lastSplit == "left":
                rightOffset += 1
                lastSplit = "right"
            else: 
                leftOffset += 1
                lastSplit = "left"
        else:
            rotationalError += 1
            rotationalError /= 2
            leftOffset = rotationalError
            rightOffset = rotationalError
            if lastSplit == "left":
                rightOffset -= 1
                lastSplit = "right"
            else: 
                leftOffset -= 1
                lastSplit = "left"
    else:
        leftOffset = rotationalError/2
        rightOffset = leftOffset
    leftOffset = -leftOffset

def rotSplit():
    global rotationalOffset
    global rotationalError
    global lastSplit
    if abs(rotationalError)%2 != 0:
        if lastSplit == "up":
            rotationalOffset = rotationalError - 1
            rotationalOffset /= 2
            lastSplit = "down"
        else: 
            rotationalOffset = rotationalError + 1
            rotationalOffset /= 2
            lastSplit = "up"
    else: rotationalOffset = rotationalError/2
    rotationalError = 0
            
def prerun():
    global startTime

    # Display message
    display.fill(0)
    display.text("Nats Testing", 0, 0, 1)
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

"""----------TIMING---------------"""
delayTime = 0.05
targetTime = 0
time1 = 0
time2 = 0
time3 = 0
time4 = 0
time5 = 0
time6 = 0
time7 = 0
time8 = 0

def calculateRunTime():
    calculatedTimes.append(((targetTime-(delayTime*8))*1000)-time1-time2-time3-time4-time5-time6-time7-time8) #post 0
    calculatedTimes.append(((targetTime-(delayTime*7))*1000)-time2-time3-time4-time5-time6-time7-time8) #post 1
    calculatedTimes.append(((targetTime-(delayTime*6))*1000)-time3-time4-time5-time6-time7-time8) #post 2
    calculatedTimes.append(((targetTime-(delayTime*5))*1000)-time4-time5-time6-time7-time8) #post 3
    calculatedTimes.append(((targetTime-(delayTime*4))*1000)-time5-time6-time7-time8) #post 4
    calculatedTimes.append(((targetTime-(delayTime*3))*1000)-time6-time7-time8) #post 5
    calculatedTimes.append(((targetTime-(delayTime*2))*1000)-time7-time8) #post 6
    calculatedTimes.append(((targetTime-(delayTime*1))*1000)-time8) #post 7

def delay(count=-1):
    global delayCount
    delayCount += 1
    if count == -1:
        time.sleep(delayTime)
    else:
        runTime = time.ticks_ms() - startTime
        error = calculatedTimes[count] - runTime
        error /= 1000
        suggestedDelay = delayTime + error
        suggestedDelay = max(0.1, min(2.5, suggestedDelay))
        time.sleep(suggestedDelay)


calculateRunTime()
prerun()

"""forward(60.0, 5750)
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
delay(0)
left()
delay(1)
forward(50.0, 5750)
delay(2)
right()
delay(3)
forward(150.0, 5750)
delay(4)
right()
delay(5)
forward(107.5, 5750)
delay(6)
right()
delay(7)
forward(46.0, 5750)"""
"""right()
delay()
right()
delay()
right()
delay()
right()
delay()
#right()
delay()
#right()
delay()
#right()
delay()
#right()
delay()"""
"""left(45)
time.sleep(1)
forward(50, 5750)
time.sleep(1)
right(45)
delay()"""

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()

delay()



endTime = time.ticks_ms()
runTime = (endTime - startTime)/1000
timeDelay = delayTime*delayCount
runTimeRaw = runTime - delayTime
neededDelay = targetTime - runTimeRaw
recommendedDelay = neededDelay/delayCount
while True: 
    display.fill(0)
    display.text("Time: " + str(runTime), 0, 0, 1)
    display.text("Delay: " + str(delayTime), 0, 16, 1)
    display.text("Delays: " + str(timeDelay), 0, 32, 1)
    display.text("Rec: " + str(recommendedDelay + delayTime), 0, 48, 1)
    display.show()
    while not button_b.check():
        pass
    display.fill(0)
    display.text("1: " + str(moveTimes[len(moveTimes)-8]), 0, 0, 1)
    display.text("2: " + str(moveTimes[len(moveTimes)-7]), 0, 16, 1)
    display.text("3: " + str(moveTimes[len(moveTimes)-6]), 0, 32, 1)
    display.text("4: " + str(moveTimes[len(moveTimes)-5]), 0, 48, 1)
    display.show()
    while not button_b.check():
        pass
    display.fill(0)
    display.text("5: " + str(moveTimes[len(moveTimes)-4]), 0, 0, 1)
    display.text("6: " + str(moveTimes[len(moveTimes)-3]), 0, 16, 1)
    display.text("7: " + str(moveTimes[len(moveTimes)-2]), 0, 32, 1)
    display.text("8: " + str(moveTimes[len(moveTimes)-1]), 0, 48, 1)
    display.show()
    while not button_b.check():
        pass
    display.fill(0)
    display.text("1: " + str(calculatedTimes[0]), 0, 0, 1)
    display.text("2: " + str(calculatedTimes[1]), 0, 16, 1)
    display.text("3: " + str(calculatedTimes[2]), 0, 32, 1)
    display.text("4: " + str(calculatedTimes[3]), 0, 48, 1)
    display.show()
    while not button_b.check():
        pass
    display.fill(0)
    display.text("5: " + str(calculatedTimes[4]), 0, 0, 1)
    display.text("6: " + str(calculatedTimes[5]), 0, 16, 1)
    display.text("7: " + str(calculatedTimes[6]), 0, 32, 1)
    display.text("8: " + str(calculatedTimes[7]), 0, 48, 1)
    display.show()
    while not button_b.check():
        pass
