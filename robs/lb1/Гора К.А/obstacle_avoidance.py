"""Braitenberg-based obstacle-avoiding robot controller."""

from controller import Robot, Compass

# Get reference to the robot.
robot = Robot()

compass = robot.getDevice("compass")
# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

#enable the compas
compass.enable(timeStep)

# Constants of the Thymio II motors and distance sensors.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 360

# Get left and right wheel motors.
leftMotor = robot.getDevice("motor.left")
rightMotor = robot.getDevice("motor.right")

# Get frontal distance sensors.
outerLeftSensor = robot.getDevice("prox.horizontal.0")
centralLeftSensor = robot.getDevice("prox.horizontal.1")
centralSensor = robot.getDevice("prox.horizontal.2")
centralRightSensor = robot.getDevice("prox.horizontal.3")
outerRightSensor = robot.getDevice("prox.horizontal.4")

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


while robot.step(timeStep) != -1:
    # to read values
    values = compass.getValues()
    leftMotor.setVelocity(initialVelocity)
    rightMotor.setVelocity(initialVelocity)
    # Read values from four distance sensors and calibrate.
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant
    #if there is an obstacle
    if ((centralLeftSensorValue != 0.0) or (centralSensorValue != 0.0) or (centralRightSensorValue != 0.0) or (outerRightSensorValue != 0.0) or (outerLeftSensorValue != 0.0)):
        #if obstacle in left
        if ((centralLeftSensorValue != 0.0) or (outerLeftSensorValue != 0.0)):
            rightMotor.setVelocity(-initialVelocity)
            leftMotor.setVelocity(initialVelocity)
         #if obstacle in right   
        elif ((centralRightSensorValue != 0.0) or (outerRightSensorValue != 0.0)):
            rightMotor.setVelocity(initialVelocity)
            leftMotor.setVelocity(-initialVelocity)            
        #if obstacle in everywhere
        elif ((outerLeftSensorValue != 0.0) and (centralLeftSensorValue != 0.0) and (centralSensorValue != 0.0)  and (centralRightSensorValue != 0.0) and (outerRightSensorValue != 0.0)):
            rightMotor.setVelocity(-initialVelocity)
            leftMotor.setVelocity(-initialVelocity)
        #if abstacle in between
        elif ((outerLeftSensorValue != 0.0) and (outerRightSensorValue != 0.0)):
            rightMotor.setVelocity(-initialVelocity)
            leftMotor.setVelocity(-initialVelocity/1.6)            
    #if there no obstacle
    else:
        #if driving to the left
        if (values[0] > 0):            
            rightMotor.setVelocity(initialVelocity/1.1)
        #if driving to the right
        elif (values[0] < 0):
            leftMotor.setVelocity(initialVelocity/1.1)
       