"""Sample Webots controller for the square path benchmark."""

from controller import Robot
import math

# Get pointer to the robot.
robot = Robot()

rightWheelSensor = robot.getPositionSensor('right wheel sensor')
rightWheelSensor.enable(16)

leftWheelSensor = robot.getPositionSensor('left wheel sensor')
leftWheelSensor.enable(16)


dWheel = 0.195
distWheels = 0.33
sensorValue = 0
l = math.pi * dWheel
r = math.pi * distWheels / 4
MAX_SPEED = 5

# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

# Repeat the following 4 times (once for each side).
for i in range(0, 4):
    # First set both wheels to go forward, so the robot goes straight.
    leftWheel.setPosition(1000)
    rightWheel.setPosition(1000)
    robot.step(16)

    while (rightWheelSensor.getValue()  < sensorValue + 20.7):
        if (rightWheelSensor.getValue() > sensorValue + 19.0):
            leftWheel.setVelocity(0.6 * MAX_SPEED)
            rightWheel.setVelocity(0.6 * MAX_SPEED)
        robot.step(160)
        print()

    if i == 0:
        leftWheel.setPosition(leftWheelSensor.getValue() + r/l * 2.0 * math.pi + 0.09)
        rightWheel.setPosition(rightWheelSensor.getValue() - r/l * 2.0 * math.pi - 0.03)
    elif i == 1:
        leftWheel.setPosition(leftWheelSensor.getValue() + r/l * 2.0 * math.pi + 0.09)
        rightWheel.setPosition(rightWheelSensor.getValue() - r/l * 2.0 * math.pi - 0.03)
    elif i == 2:
        leftWheel.setPosition(leftWheelSensor.getValue() + r/l * 2.0 * math.pi + 0.11)
        rightWheel.setPosition(rightWheelSensor.getValue() - r/l * 2.0 * math.pi)
    elif i == 3:
        continue
    robot.step(1000)
    
    sensorValue = rightWheelSensor.getValue() 

    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)

# Stop the robot when path is completed, as the robot performance
# is only computed when the robot has stopped.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
