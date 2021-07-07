from controller import Robot

# Get pointer to the robot.
robot = Robot()

# Get pointer to each wheel of our robot.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

#Get right wheel sensor
rightWheelSensor = robot.getPositionSensor('right wheel sensor')
rightWheelSensor.enable(16)
#Get left wheel sensor
leftWheelSensor = robot.getPositionSensor('left wheel sensor')
leftWheelSensor.enable(16)
# Текущее значение датчика положения колес
sensorValue = 0
# Радиус колеса 
wheel_radius = 0.195/2
MAX_SPEED = 5.24
# Repeat the following 4 times (once for each side).
for i in range(0, 4):  
    # First set both wheels to go forward, so the robot goes straight.
    leftWheel.setPosition(1000)
    rightWheel.setPosition(1000)
    robot.step(16)
    
    # Двигаться пока текущее положение не будет равно двум метрам + положение пред. вершины
    while ((rightWheelSensor.getValue() * wheel_radius) < (2.0 + sensorValue)):
        # Снижаем скорость почти достигнув вершины
        if ((rightWheelSensor.getValue() * wheel_radius) > (1.7 + sensorValue)):
            leftWheel.setVelocity(0.7 * MAX_SPEED)
            rightWheel.setVelocity(0.7 * MAX_SPEED)
        robot.step(16)
    
    # При повороте в зависимости от вершины меняем позицию колеса
    if i == 0:
        leftWheel.setPosition(leftWheelSensor.getValue() + 2.75)
        rightWheel.setPosition(rightWheelSensor.getValue() - 2.71)
        robot.step(930)
    if i == 1:
        leftWheel.setPosition(leftWheelSensor.getValue() + 2.74) #изменяем значение с учетом погрешности
        rightWheel.setPosition(rightWheelSensor.getValue() - 2.70)
        robot.step(930)
    if i == 2:
        leftWheel.setPosition(leftWheelSensor.getValue() + 2.75)
        rightWheel.setPosition(rightWheelSensor.getValue() - 2.71)
        robot.step(930)
    
    
    #Ставим исходную скорость 
    leftWheel.setVelocity(MAX_SPEED)
    rightWheel.setVelocity(MAX_SPEED)
    
    #Снова Считаем текущую позицию датчика
    sensorValue = rightWheelSensor.getValue() * wheel_radius
    
# Stop the robot when path is completed, as the robot performance
# is only computed when the robot has stopped.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)