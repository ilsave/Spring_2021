"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot

# import Compass module
from controller import Compass
  
# Get reference to the robot.
robot = Robot()

kp = 0.4
kk = 0.2
# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# get robot's Compass device
compass = robot.getCompass("compass")
# enable the Compass
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
initialVelocity = 0.98 * maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(initialVelocity)
rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
    # Read values from four distance sensors and calibrate.
    values = compass.getValues()
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant
    val=[outerLeftSensorValue,centralLeftSensorValue,centralSensorValue,centralRightSensorValue,outerRightSensorValue]

    # Set wheel velocities based on sensor values, prefer right turns if the central sensor is triggered.
    if(outerLeftSensorValue!=0)or(centralLeftSensorValue!=0)or(centralSensorValue!=0)or(centralRightSensorValue!=0)or(outerRightSensorValue!=0):
        lm = (initialVelocity * (1-centralRightSensorValue*kp) * (1 - outerRightSensorValue*kp) )
        rm = (initialVelocity * (1-centralLeftSensorValue*kp) * (1 - outerLeftSensorValue*kp) * (1 - centralSensorValue*kp))
        if((1-centralRightSensorValue*kp)<0)and((1 - outerRightSensorValue*kp)<0):
            lm=-lm
        if(((1-centralLeftSensorValue*kp)<0)or((1 - outerLeftSensorValue*kp)<0)or((1 - centralSensorValue*kp)<0))and(rm>0):
            rm=-rm
        if(lm<-9.53):
            leftMotor.setVelocity(-9.53)
        else:
            leftMotor.setVelocity(lm)
        if(rm<-9.53):
            rightMotor.setVelocity(-9.53)
        else:
            rightMotor.setVelocity(rm)
    else:
        
        if(values[0]>0.0001):
            rightMotor.setVelocity(initialVelocity * (1 - (values[0])*kk))
            leftMotor.setVelocity(initialVelocity)
        elif(values[0]<-0.0001):
            leftMotor.setVelocity(initialVelocity * (1 + (values[0])*kk))
            rightMotor.setVelocity(initialVelocity)
        else:
            leftMotor.setVelocity(initialVelocity)
            rightMotor.setVelocity(initialVelocity)