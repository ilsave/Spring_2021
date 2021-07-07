"""Sample Webots controller for the square path benchmark."""

from controller import Robot

# Get pointer to the robot.
robot = Robot()

# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

rightWheelSensor = robot.getPositionSensor('right wheel sensor')
rightWheelSensor.enable(16) # Refreshes the sensor every 16ms.

rSensorValue = 0

# Repeat the following 4 times (once for each side).
for i in range(0, 4):
    # First set both wheels to go forward, so the robot goes straight.
    leftWheel.setVelocity(5)
    rightWheel.setVelocity(5)
    leftWheel.setPosition(1000)
    rightWheel.setPosition(1000)
    robot.step(1)
    rSensorValue = rightWheelSensor.getValue()
    while(rightWheelSensor.getValue() - rSensorValue < 20.5): #20.7
        print(rightWheelSensor.getValue())
        robot.step(1)
    
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

    leftWheel.setVelocity(3)
    rightWheel.setVelocity(3)
    leftWheel.setPosition(1000)
    rightWheel.setPosition(-1000)
    rSensorValue = rightWheelSensor.getValue()
    while((rightWheelSensor.getValue() - rSensorValue) > -2.6874): #2.66
        robot.step(1)
    rSensorValue = rightWheelSensor.getValue();
    
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    robot.step(100)

leftWheel.setVelocity(0)
rightWheel.setVelocity(0)