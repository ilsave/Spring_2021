"""square_path controller."""

from controller import Robot
# Максимальная скорость робота
MAX_VEL = 5.24

# Получить доступ к роботу
robot = Robot()

# Получить доступ к двигателям левого и правого колес.
leftWheel = robot.getMotor('left wheel')
rightWheel = robot.getMotor('right wheel')

# Получить доступ к сенсорному датчику правого колеса
rightWheelSensor = robot.getPositionSensor('right wheel sensor')
rightWheelSensor.enable(16)
# Получить доступ к сенсорному датчику левого колеса
leftWheelSensor = robot.getPositionSensor('left wheel sensor')
leftWheelSensor.enable(16)
# Текущее значение датчика положения колес
positionSensorValue = 0

# Радиус колеса 
wheel_radius = 0.195/2
# Расстояние между колесами
wheel_spacing = 0.33

# Повторение следующих действий 4 раза (по одному разу для каждой стороны).
for i in range(0, 4):
    
    # Для первоночальных значений датчика
    leftWheel.setPosition(1000)
    rightWheel.setPosition(1000)
    robot.step(16)
    
    # Двигаться пока текущее положение не будет равно двум метрам + положение пред. вершины
    while rightWheelSensor.getValue() * wheel_radius < 2.0 + positionSensorValue:
        # Снижаем скорость почти достигнув вершины
        if rightWheelSensor.getValue() * wheel_radius > 1.9 + positionSensorValue:
            leftWheel.setVelocity(0.6 * MAX_VEL)
            rightWheel.setVelocity(0.6 * MAX_VEL)
        robot.step(160)
    
    # При повороте в зависимости от вершины меняем позицию колеса
    if i == 0:
        leftWheel.setPosition(leftWheelSensor.getValue() + 2.75)
        rightWheel.setPosition(rightWheelSensor.getValue() - 2.71)
        robot.step(912)
    if i == 1:
        leftWheel.setPosition(leftWheelSensor.getValue() + 2.72)
        rightWheel.setPosition(rightWheelSensor.getValue() - 2.71)
        robot.step(912)
    if i == 2:
        leftWheel.setPosition(leftWheelSensor.getValue() + 2.77)
        rightWheel.setPosition(rightWheelSensor.getValue() - 2.66)
        robot.step(912)
    if i == 3:
        break
    
    # Возвращаем скорость 
    leftWheel.setVelocity(MAX_VEL)
    rightWheel.setVelocity(MAX_VEL)
    
    # Считываем тек. позицию на датчике
    positionSensorValue = rightWheelSensor.getValue() * wheel_radius
    
# Остановить робота, когда путь будет завершен, так как производительность робота
# вычисляется только тогда, когда робот остановился.
leftWheel.setVelocity(0)
rightWheel.setVelocity(0)
