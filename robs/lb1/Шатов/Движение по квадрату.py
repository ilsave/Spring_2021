"""Sample Webots controller for the square path benchmark."""

from controller import Robot

# Get pointer to the robot.
robot = Robot()


# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')


#Get right wheel sensor
rightWheelSensor = robot.getPositionSensor('right wheel sensor')
rightWheelSensor.enable(16)

   
# Repeat the following 4 times (once for each side).
for i in range(0, 4):
    var = 0 
    # First set both wheels to go forward, so the robot goes straight.
    leftWheel.setPosition(1000)
    rightWheel.setPosition(1000)
    print(rightWheelSensor.getValue()/(i+1))
    if i == 0:
        while round(rightWheelSensor.getValue(),1)  != 20.6:
    # Wait for the robot to reach a corner.
            print(rightWheelSensor.getValue()/(i+1))
            robot.step(var)
            var += 1
    else:
        while round(rightWheelSensor.getValue()-(18*i),1)  < 20.7:
    # Wait for the robot to reach a corner.
            print(rightWheelSensor.getValue()/(i+1))
            robot.step(var)
            var += 1
    # Then, set the right wheel backward, so the robot will turn right.
    leftWheel.setPosition(1000)
    rightWheel.setPosition(-1000)
    # Wait until the robot has turned 90 degrees clockwise.
    if i == 2 or i == 0:
        robot.step(464)
    else:
        robot.step(465)

# Stop the robot when path is completed, as the robot performance
# is only computed when the robot has stopped.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)