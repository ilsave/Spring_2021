"""Sample Webots controller for the square path benchmark."""

from controller import Robot

# Get pointer to the robot.
robot = Robot()

# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

leftWheelSensor = robot.getPositionSensor('left wheel sensor')
leftWheelSensor.enable(16)

# Repeat the following 4 times (once for each side).
# 20.445
# 22.874
# 57.2958
# 5.24
length = 20.51
step = 0.083
turn = 2.447
count = 0
spin = length + turn
way = length
for i in range(0, 4):

    leftWheel.setPosition(1000)
    rightWheel.setPosition(1000)
    
    robot.step(16)
    
    while leftWheelSensor.getValue() < way:
        
        if i != 0:
            if count == 15:
                rightWheel.setVelocity(2.62)
                if i == 1:
                    robot.step(16)
                if i == 2:
                    robot.step(16)
                    robot.step(16)
                if i == 3:
                    rightWheel.setVelocity(1.72)
                    robot.step(16)
                    robot.step(16)
            if count == 195:
                if i == 3:
                    rightWheel.setVelocity(1.72)
                    robot.step(16)
                    robot.step(16)
        print(count)
        robot.step(16)
        rightWheel.setVelocity(5.24)
        count+=1
    
    if i == 2:
        way = way - 0.19
        
    if i == 3:
        break
    
    
    leftWheel.setPosition(1000)
    rightWheel.setPosition(-1000)
    
    while leftWheelSensor.getValue() < spin:
        robot.step(16)
    
    count = 0
    way = way + length + turn
    spin = way + turn
    
    if i == 0:
        way = way + 0.2
        spin = spin + 0.2
    
    if i == 1:
        way = way + 0.19
        spin = spin + 0.13
        
    if i == 2:
        way = way + 0.4
    

# Stop the robot when path is completed, as the robot performance
# is only computed when the robot has stopped.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
