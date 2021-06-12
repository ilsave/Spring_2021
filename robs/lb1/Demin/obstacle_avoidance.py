"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot
from controller import Compass

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 150

# Get left and right wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

compass = robot.getCompass("compass")
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
initialVelocity = 1 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
    
    values = compass.getValues()
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant
    
#    if outerLeftSensorValue != 0 or centralLeftSensorValue != 0 or centralSensorValue != 0 or centralRightSensorValue != 0 or outerRightSensorValue != 0:
#        initialVelocity = 0.75 * maxMotorVelocity
#    else:
#        initialVelocity = 1 * maxMotorVelocity

    if outerLeftSensorValue != 0 or centralLeftSensorValue != 0 or centralSensorValue != 0 or centralRightSensorValue != 0 or outerRightSensorValue != 0:            
        # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
        leftMotor.setVelocity(initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 2 - centralSensorValue)
        rightMotor.setVelocity(initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 2)
        robot.step(timeStep)
        robot.step(timeStep)
        robot.step(timeStep)
    else:
        if values[2] > -1:
            if values[0] > 0:
                leftMotor.setVelocity(initialVelocity)
                rightMotor.setVelocity(8.0)
            if values[0] < 0:
                rightMotor.setVelocity(initialVelocity)
                leftMotor.setVelocity(8.0)
    
