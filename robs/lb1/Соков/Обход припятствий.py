"""Braitenberg-based obstacle-avoiding robot controller."""
from controller import Compass 

from controller import Robot
#from controller import Compass
 
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
initialVelocity = maxMotorVelocity * 0.95

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    values = compass.getValues()
    outerLeftSensorValue = outerLeftSensor.getValue() / 2 * distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / 2 * distanceSensorCalibrationConstant
    example = values[0]
    example = example - 0.07
    example *= 10
    print(example)
    
    leftMotor.setVelocity(initialVelocity +  example /1.5 - (centralRightSensorValue + outerRightSensorValue)/1.5)
    rightMotor.setVelocity(initialVelocity -  example /1.5 - (centralLeftSensorValue + outerLeftSensorValue)/1.5)