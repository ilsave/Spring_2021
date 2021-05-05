"""Sample Webots controller for the square path benchmark."""

from controller import Robot

# Get pointer to the robot.
robot = Robot()

# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

rightWheelSensor = robot.getPositionSensor('right wheel sensor')
rightWheelSensor.enable(1) # Refreshes the sensor every 1ms.

leftWheelSensor = robot.getPositionSensor('left wheel sensor')
leftWheelSensor.enable(1) # Refreshes the sensor every 1ms.

robot.step(1)
temp = rightWheelSensor.getValue()


# Repeat the following 4 times (once for each side).
for i in range(0, 4):
    leftWheel.setVelocity(5.24)
    rightWheel.setVelocity(5.24)
    leftWheel.setPosition(5000)
    rightWheel.setPosition(5000)
    
    while ((rightWheelSensor.getValue() - temp) < 20.7):
        robot.step(1)
        
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    robot.step(50)
    
    print(rightWheelSensor.getValue() - temp)
    
    
    leftWheel.setVelocity(3)
    rightWheel.setVelocity(3)
    leftWheel.setPosition(1000)
    rightWheel.setPosition(-1000)
    
    temp2 = leftWheelSensor.getValue()
    
    while ((leftWheelSensor.getValue() - temp2) < 2.68252742707):
        robot.step(1)
        
    
    temp = rightWheelSensor.getValue()
    
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    robot.step(50)

# Stop the robot when path is completed, as the robot performance
# is only computed when the robot has stopped.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
