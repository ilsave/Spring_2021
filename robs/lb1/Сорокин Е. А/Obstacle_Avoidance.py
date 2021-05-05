"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot

# Get reference to the robot.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 200 #360 

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
initialVelocity = 0.7 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)


# import Compass module
from controller import Compass
# get robot's Compass device
compass = robot.getCompass("compass")
# enable the Compass
compass.enable(timeStep)

right = 0
left = 0
values = 0


while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant

    # to read values
    values = compass.getValues()
    
    if ((outerRightSensor.getValue() == 0) and (values[0] > 0)):
        right = values[0]*4
    elif ((outerLeftSensor.getValue() == 0) and (values[0] < 0)):
        left = -values[0]*4
    elif((centralSensor.getValue() != 0)):
        if(values[0] > 0):
            right += values[0]*2
        elif(values[0] < 0):
            left += -values[0]*2
    
    leftMotor.setVelocity(maxMotorVelocity - left)
    rightMotor.setVelocity(maxMotorVelocity - right)
    
    if ((centralRightSensorValue != 0) or (outerRightSensorValue != 0)):
        leftMotor.setVelocity(initialVelocity - (centralRightSensorValue + outerRightSensorValue) / 2)
    elif ((centralLeftSensorValue != 0) or (outerLeftSensorValue != 0)):
        rightMotor.setVelocity(initialVelocity - (centralLeftSensorValue + outerLeftSensorValue) / 2)
      
    
    
    