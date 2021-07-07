"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot
from controller import Compass

# Get reference to the robot.
robot = Robot()
compass = robot.getCompass("compass")

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())
compass.enable(timeStep)

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 360

# Get left and right wheel motors.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

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
initialVelocity = 0.8 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(maxMotorVelocity)
rightMotor.setVelocity(maxMotorVelocity)

while robot.step(timeStep) != -1:
    # print(rightMotor.getVelocity())
    direction = round(compass.getValues()[0], 4)
    #print(direction)
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant

    if(abs(direction) > 0.001):
        print(abs(direction))
        leftMotor.setVelocity(initialVelocity + direction*1.5)
        rightMotor.setVelocity(initialVelocity - direction*1.5)
    
    if(abs(direction) < 0.01):
        # print(abs(direction))
        leftMotor.setVelocity(maxMotorVelocity)
        rightMotor.setVelocity(maxMotorVelocity)

    # if(outerLeftSensorValue != 0 or centralLeftSensorValue != 0 or centralSensorValue != 0 ):
    #     if(centralLeftSensorValue != 0 and centralSensorValue != 0 and centralLeftSensorValue>centralRightSensorValue):
    #         leftMotor.setVelocity(maxMotorVelocity)
    #         rightMotor.setVelocity(-maxMotorVelocity)
    #     if(centralRightSensorValue != 0 and centralSensorValue != 0 and entralRightSensorValue > centralLeftSensorValue):
    #         leftMotor.setVelocity(-maxMotorVelocity)
    #         rightMotor.setVelocity(maxMotorVelocity)
    #     leftMotor.setVelocity(maxMotorVelocity)
    #     rightMotor.setVelocity(0)

    if(outerLeftSensorValue != 0 or centralLeftSensorValue != 0 or centralSensorValue != 0 ):
        if(centralLeftSensorValue != 0 and centralSensorValue != 0 and centralLeftSensorValue>centralRightSensorValue):
            leftMotor.setVelocity(maxMotorVelocity)
            rightMotor.setVelocity(-maxMotorVelocity)
        if(centralRightSensorValue != 0 and centralSensorValue != 0 and entralRightSensorValue > centralLeftSensorValue):
            leftMotor.setVelocity(-maxMotorVelocity)
            rightMotor.setVelocity(maxMotorVelocity)
        else:
            leftMotor.setVelocity(maxMotorVelocity)
            rightMotor.setVelocity(-maxMotorVelocity)
            
        leftMotor.setVelocity(maxMotorVelocity)
        rightMotor.setVelocity(0)
        
    if(outerRightSensorValue != 0 or centralRightSensorValue != 0 ):
        leftMotor.setVelocity(0)
        rightMotor.setVelocity(maxMotorVelocity)
        