from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

max_speed = 6.28
current_speed = 0.0
acceleration = 0.1 

proximity_sensors = []
sensor0 = robot.getDevice('ps0')
sensor0.enable(timestep)
proximity_sensors.append(sensor0)

sensor7 = robot.getDevice('ps7')
sensor7.enable(timestep)
proximity_sensors.append(sensor7)

while robot.step(timestep) != -1:
    sensor_values = [sensor.getValue() for sensor in proximity_sensors]
    print("Sensor ps0 value:", sensor_values[0])
    print("Sensor ps7 value:", sensor_values[1])
    if sensor_values[0] > 80 or sensor_values[1] > 80:  
        current_speed = 0
    else:
        if current_speed < max_speed:
            current_speed += acceleration  
            if current_speed > max_speed:  
                current_speed = max_speed
    left_motor.setVelocity(current_speed)
    right_motor.setVelocity(current_speed)