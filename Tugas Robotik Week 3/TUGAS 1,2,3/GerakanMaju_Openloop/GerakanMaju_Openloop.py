from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

speed = 2.0  

while robot.step(timestep) != -1:
    if speed < 6.28:  
        speed += 0.05  
        if speed > 6.28:  
            speed = 6.28 

    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed)