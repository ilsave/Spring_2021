"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot, Compass

# Get reference to the robot.
robot = Robot()

# get robot's Compass device
compass = robot.getCompass("compass")

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

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

# enable the Compass
compass.enable(timeStep)

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Set ideal motor velocity.
initialVelocity = maxMotorVelocity

# Set the initial velocity of the left and right wheel motors.
#leftMotor.setVelocity(initialVelocity)
#rightMotor.setVelocity(initialVelocity)

while robot.step(timeStep) != -1:
    # to read values
    values = compass.getValues()
    
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant
    
    #if there is an obstacle
    if centralLeftSensorValue != 0.0 or centralSensorValue != 0.0 or centralRightSensorValue != 0.0 or outerRightSensorValue != 0.0 or outerLeftSensorValue != 0.0:
        #if obstacle in left
        if centralLeftSensorValue != 0.0 or centralSensorValue != 0.0 or outerLeftSensorValue != 0.0:
            rightMotor.setVelocity(-initialVelocity)
            leftMotor.setVelocity(initialVelocity)
         #if obstacle in right   
        elif centralSensorValue != 0.0 or centralRightSensorValue != 0.0 or outerRightSensorValue != 0.0:
            leftMotor.setVelocity(-initialVelocity)
            rightMotor.setVelocity(initialVelocity)
        #if obstacle in everywhere
        elif centralLeftSensorValue != 0.0 and  outerLeftSensorValue != 0.0 and centralRightSensorValue != 0.0 and outerRightSensorValue != 0.0:
            leftMotor.setVelocity(-initialVelocity)
            rightMotor.setVelocity(-initialVelocity)
    #if there isnt an obstacle
    else:
        #displacement of the vision towards the line
        if (round(values[0],3) > 0.001) and (centralLeftSensorValue == 0.0 and centralSensorValue == 0.0 and outerLeftSensorValue == 0.0 and centralRightSensorValue == 0.0 and outerRightSensorValue == 0.0):
            rightMotor.setVelocity(initialVelocity/1.1)
            leftMotor.setVelocity(initialVelocity)
        elif (round(values[0],3) < 0.001) and (centralLeftSensorValue == 0.0 and centralSensorValue == 0.0 and outerLeftSensorValue == 0.0 and centralRightSensorValue == 0.0 and outerRightSensorValue == 0.0):
            leftMotor.setVelocity(initialVelocity/1.1)
            rightMotor.setVelocity(initialVelocity)
        #if everything is alright then just move
        else:
            leftMotor.setVelocity(initialVelocity)
            rightMotor.setVelocity(initialVelocity)