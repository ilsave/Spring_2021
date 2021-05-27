"""Sample Webots controller for the square path benchmark."""
from controller import Robot

# Get pointer to the robot.
robot = Robot()

# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

rightWheelSensor = robot.getPositionSensor('right wheel sensor')
rightWheelSensor.enable(16) # Refreshes the sensor every 16ms.

leftWheelSensor = robot.getPositionSensor('left wheel sensor')
leftWheelSensor.enable(16) # Refreshes the sensor every 16ms.

counter = 1
currDistance = 20.7
leftWheel.setPosition(100)
rightWheel.setPosition(100)
# Repeat the following 4 times (once for each side).
for i in range(0, 4):
    # First set both wheels to go forward, so the robot goes straight.
    leftWheel.setVelocity(4)
    rightWheel.setVelocity(4)
    # Wait for the robot to reach a corner.
    
    distance = 0
    while 1:
        distance = leftWheelSensor.getValue()
        print(distance, "  ", currDistance * counter)
        if distance  - (2.633 * (counter - 1)) >= currDistance * counter:
            break
        robot.step(1)
    
    # Then, set the right wheel backward, so the robot will turn right.
    if counter == 4:
        break
    print('corner')
    leftWheel.setPosition(0)
    rightWheel.setPosition(0)
   
    leftWheel.setVelocity(2)
    rightWheel.setVelocity(2)
    leftWheel.setPosition(100)
    rightWheel.setPosition(-100)
    const = (currDistance + 2.633) * counter
    while 1:
        distance = leftWheelSensor.getValue()
        print(distance)
        if distance >= const:
            break
        robot.step(1)
    
    leftWheel.setVelocity(0)
    counter += 1
    leftWheel.setPosition(100)
    rightWheel.setPosition(100)
    # Wait until the robot has turned 90 degrees clockwise.
    

# Stop the robot when path is completed, as the robot performance
# is only computed when the robot has stopped.
leftWheel.setPosition(0)
rightWheel.setPosition(0)
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
