"""Braitenberg-based obstacle-avoiding robot controller."""


# import Compass module
from controller import Compass

from controller import Robot

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 15
distanceSensorCalibrationConstant = 200

# Get left and right wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

# get robot's Compass device
compass = robot.getCompass("compass")
# enable the Compass
compass.enable(timeStep)

# Get frontal distance sensors.
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Enable distance sensors.
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = 0.7 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

# flag = False

while robot.step(timeStep) != -1:
    compassValues = compass.getValues()
    print(compassValues)
    leftMotor.setVelocity(initialVelocity)
    rightMotor.setVelocity(initialVelocity)
    
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant

    compassLeft, _, compassBack = compass.getValues()
    if (outerLeftSensorValue != 0 or centralLeftSensorValue != 0 or centralSensorValue != 0 or centralRightSensorValue != 0 or outerRightSensorValue != 0):
        # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
        leftMotor.setVelocity(initialVelocity - (centralRightSensorValue + outerRightSensorValue) - centralSensorValue)
        rightMotor.setVelocity(initialVelocity - (centralLeftSensorValue + outerLeftSensorValue))
    
    elif (compassBack > -0.9):
        leftMotor.setVelocity(initialVelocity * (1 + compassLeft/2))
        rightMotor.setVelocity(initialVelocity * (1 - compassLeft/2))
        compassLeft, _, compassBack = compass.getValues()
    elif (compassBack > -0.999):
        leftMotor.setVelocity(initialVelocity * (1 + compassLeft))
        rightMotor.setVelocity(initialVelocity * (1 - compassLeft))
        compassLeft, _, compassBack = compass.getValues()