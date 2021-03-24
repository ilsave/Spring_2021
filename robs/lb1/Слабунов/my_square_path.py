"""my_square_path controller."""

"""Sample Webots controller for the square path benchmark."""

from controller import Robot
import math

# Get pointer to the robot.
robot = Robot()

# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

# Получаем доступ к датчику положения правого колеса
rightWheelSensor = robot.getPositionSensor('right wheel sensor')
rightWheelSensor.enable(16)
# Получаем доступ к датчику положения левого колеса
leftWheelSensor = robot.getPositionSensor('left wheel sensor')
leftWheelSensor.enable(16)

# Диаметр колеса (в метрах)
diamWheel = 0.195
# Расстояние между колесами (в метрах)
distWheels = 0.33
# Текущее значение датчика положения колес (в метрах)
sensorValue = 0
# Расстояние, пройденное роботом за один оборот
l = math.pi * diamWheel
# Max velocity
MAX_SPEED = 5.24

# Repeat the following 4 times (once for each side).
for i in range(0, 4):

    leftWheel.setPosition(1000)
    rightWheel.setPosition(1000)
    robot.step(16)

    while rightWheelSensor.getValue() * diamWheel/2.0 < sensorValue + 2.0:
        if rightWheelSensor.getValue() * diamWheel/2.0 > sensorValue + 1.9:
            leftWheel.setVelocity(0.6 * MAX_SPEED)
            rightWheel.setVelocity(0.6 * MAX_SPEED)
        robot.step(160)

    if i == 0:
        leftWheel.setPosition(leftWheelSensor.getValue() + (math.pi/2.0 * distWheels/2.0)/l * 2.0 * math.pi + 0.09)
        rightWheel.setPosition(rightWheelSensor.getValue() - (math.pi/2.0 * distWheels/2.0)/l * 2.0 * math.pi - 0.05)
    elif i == 1:
        leftWheel.setPosition(leftWheelSensor.getValue() + (math.pi/2.0 * distWheels/2.0)/l * 2.0 * math.pi + 0.06)
        rightWheel.setPosition(rightWheelSensor.getValue() - (math.pi/2.0 * distWheels/2.0)/l * 2.0 * math.pi - 0.05)
    elif i == 2:
        leftWheel.setPosition(leftWheelSensor.getValue() + (math.pi/2.0 * distWheels/2.0)/l * 2.0 * math.pi + 0.11)
        rightWheel.setPosition(rightWheelSensor.getValue() - (math.pi/2.0 * distWheels/2.0)/l * 2.0 * math.pi)
    elif i == 3:
        continue
    robot.step(912)
    
    sensorValue = rightWheelSensor.getValue() * diamWheel/2

    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)

# Stop the robot when path is completed, as the robot performance
# is only computed when the robot has stopped.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
