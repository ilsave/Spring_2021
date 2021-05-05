"""Sample Webots controller for the square path benchmark."""

from controller import Robot

# Get pointer to the robot.
robot = Robot()

# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

rightWheelSensor = robot.getPositionSensor('right wheel sensor')
rightWheelSensor.enable(1) # Refreshes the sensor every 16ms.

val = 0
print(rightWheelSensor.getValue())
leftWheel.setVelocity(5)
rightWheel.setVelocity(5)
leftWheel.setPosition(5000)
rightWheel.setPosition(5000)
robot.step(100)


# First set both wheels to go forward, so the robot goes straight.
leftWheel.setVelocity(5.24)
rightWheel.setVelocity(5.24)
leftWheel.setPosition(5000)
rightWheel.setPosition(5000)
val = 20.7
# Wait for the robot to reach a corner.
while (rightWheelSensor.getValue() < val):
    robot.step(1)
print(rightWheelSensor.getValue())

leftWheel.setVelocity(0)
rightWheel.setVelocity(0)

# Repeat the following 4 times (once for each side).
for i in range(0, 3):
    robot.step(500)
    # Then, set the right wheel backward, so the robot will turn right.
    leftWheel.setVelocity(5)
    rightWheel.setVelocity(5)
    leftWheel.setPosition(5000)
    rightWheel.setPosition(-5000)
    val -= 2.63 # (2.61 + 0.023 * (i%2)) # 2.59
    # Wait until the robot has turned 90 degrees clockwise.
    while (rightWheelSensor.getValue() > val):
        robot.step(1)
    print(rightWheelSensor.getValue())
    
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    robot.step(70)
    
    # First set both wheels to go forward, so the robot goes straight.
    leftWheel.setVelocity(5)
    rightWheel.setVelocity(5)
    leftWheel.setPosition(5000)
    rightWheel.setPosition(5000)
    val += 20.65
    # Wait for the robot to reach a corner.
    while (rightWheelSensor.getValue() < val):
        robot.step(1)
    print(rightWheelSensor.getValue())
    
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    

# Stop the robot when path is completed, as the robot performance
# is only computed when the robot has stopped.
# leftWheel.setVelocity(0)
# rightWheel.setVelocity(0)
