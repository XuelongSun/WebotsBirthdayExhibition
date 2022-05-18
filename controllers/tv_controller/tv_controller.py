"""tv_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from cmath import phase
from dis import dis
from controller import Robot
import numpy as np
import cv2


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

display = robot.getDevice('display')
dis_width = display.getWidth()
dis_height = display.getHeight()



motors = []
for i in range(1, 5):
    motors.append(robot.getDevice('motor' + str(i)))
    motors[-1].setPosition(np.inf)
    motors[-1].setVelocity(5)
            
def update_display(img):
    ir = display.imageNew(img.tobytes(),
                        display.RGB,
                        width=dis_width,
                        height=dis_height)
    display.imagePaste(ir, 0, 0, False)
    display.imageDelete(ir)
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

str_custom = robot.getCustomData().split(' ')

v_name = str_custom[0]
video = cv2.VideoCapture(v_name)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    str_custom = robot.getCustomData().split(' ')
    v_name = str_custom[0]
    current_state = str_custom[1]
    if current_state == 'play':
        ret, frame = video.read()
        if frame is None:
            video = cv2.VideoCapture(v_name)
            ret, frame = video.read()
        frame = cv2.resize(frame, (dis_width, dis_height))
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        update_display(frame)
        motors[0].setVelocity(int(str_custom[3]))
        motors[1].setVelocity(int(str_custom[2]))
        motors[2].setVelocity(int(str_custom[2]))
        motors[3].setVelocity(int(str_custom[3]))
    elif current_state == 'wait':
        for motor in motors:
            motor.setVelocity(0)
    else:
        for motor in motors:
            motor.setVelocity(0)
        pass
