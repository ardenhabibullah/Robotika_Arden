from controller import Robot

robot = Robot()

timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

max_speed = 6.28 
left_speed = 4.5
right_speed = 5.5

while robot.step(timestep) != -1:
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)