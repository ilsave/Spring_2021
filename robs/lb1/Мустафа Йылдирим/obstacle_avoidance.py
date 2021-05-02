"""avoid_obstacles controller."""

from controller import Robot, Compass

# Получить доступ к роботу
robot = Robot()

# Получить длину шага моделирования.
timeStep = int(robot.getBasicTimeStep())

# Константы двигателей Thymio II и датчиков расстояния.
maxMotorVelocity = 9.53
distanceSensorCalibrationConstant = 360

# Получить доступ к двигателям левого и правого колес.
leftMotor = robot.getMotor("motor.left")
rightMotor = robot.getMotor("motor.right")

# Получить доступ к фронтальным датчикам расстояния
outerLeftSensor = robot.getDistanceSensor("prox.horizontal.0")
centralLeftSensor = robot.getDistanceSensor("prox.horizontal.1")
centralSensor = robot.getDistanceSensor("prox.horizontal.2")
centralRightSensor = robot.getDistanceSensor("prox.horizontal.3")
outerRightSensor = robot.getDistanceSensor("prox.horizontal.4")

# Включить датчики расстояний
outerLeftSensor.enable(timeStep)
centralLeftSensor.enable(timeStep)
centralSensor.enable(timeStep)
centralRightSensor.enable(timeStep)
outerRightSensor.enable(timeStep)

# Отключите режим ПИД-управление двигателем
# (чтобы робот двигался бесконечно)
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# Установить начальную скорость левого и правого колеса (без ограничений)
leftMotor.setVelocity(maxMotorVelocity)
rightMotor.setVelocity(maxMotorVelocity)

# Подключить компас
compass = robot.getCompass("compass")
# Включить компас
compass.enable(timeStep)

# Основной цикл с шагом симуляции
while robot.step(timeStep) != -1:
    
    # Считывание значений с четырех датчиков расстояния и калибровка
    outerLeftSensorValue = outerLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralLeftSensorValue = centralLeftSensor.getValue() / distanceSensorCalibrationConstant
    centralSensorValue = centralSensor.getValue() / distanceSensorCalibrationConstant
    centralRightSensorValue = centralRightSensor.getValue() / distanceSensorCalibrationConstant
    outerRightSensorValue = outerRightSensor.getValue() / distanceSensorCalibrationConstant
    
    # Считывание значений с компасса
    values = compass.getValues()
    
    # Если препятствия не обнаружены, мы считываем значение с компаса
    if outerLeftSensorValue == 0 and centralLeftSensorValue == 0 and centralSensorValue == 0 and centralRightSensorValue == 0 and outerRightSensorValue == 0:
        # Если значение показывает < -0.01, значит немного уменьшаем скорость у левого колеса 
        if values[0] < -0.01:
            leftMotor.setVelocity(0.85 * maxMotorVelocity)
            rightMotor.setVelocity(maxMotorVelocity)
        # Если значение показывает > 0.01, значит немного уменьшаем скорость у правого колеса 
        elif values[0] > 0.01:
            leftMotor.setVelocity(maxMotorVelocity)
            rightMotor.setVelocity(0.85 * maxMotorVelocity)
        # Иначе продолжаем двигаться
        else:
            leftMotor.setVelocity(maxMotorVelocity)
            rightMotor.setVelocity(maxMotorVelocity)      
            
    # Если препятствия обнаружены, мы считываем значения с датчиков
    # Если левые или центральный датчики показывают ненулевые значения,
    # значит снижаем скорость у правого колеса
    if outerLeftSensorValue != 0 or centralLeftSensorValue != 0 or centralSensorValue != 0:
        leftMotor.setVelocity(maxMotorVelocity)
        rightMotor.setVelocity(-0.7 * maxMotorVelocity) 
    # Если правые датчики показывают ненулевые значения,
    # значит снижаем скорость у левого колеса
    if outerRightSensorValue != 0 or centralRightSensorValue != 0:
        leftMotor.setVelocity(-0.7 * maxMotorVelocity)
        rightMotor.setVelocity(maxMotorVelocity)
