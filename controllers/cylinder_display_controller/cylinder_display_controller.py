"""cylinder_display_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from cmath import phase
from controller import Robot
import numpy as np
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motor = robot.getDevice('motor')

motor.setPosition(np.inf)

period = 60

c = np.random.choice([0, 1])
bg = [0xFF80FF, 0x8080FF][c]

display = robot.getDevice('display')
dis_width = display.getWidth()
dis_height = display.getHeight()
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
def clean_display():
    display.setColor(bg)
    # display.setOpacity(0.1)
    display.fillRectangle(0, 0, dis_width, dis_height)

display_strs = ['生日快乐', 'O(∩_∩)O', '永远18', '天下最美',
                '天天开心', '梦想成真', '美好常伴', '(￣▽￣)']
loop_counter = 0

phase_number = 3
# display.fillRectangle(1, 1, 20, 4)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
s = display_strs[np.random.randint(0, len(display_strs))]
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    current_state = robot.getCustomData()
    if current_state == 'play':
        motor.setVelocity(-np.pi*2/(period*timestep)*1000)
        current_phase = (loop_counter // period) % phase_number
        if loop_counter % period == 0:
            s = display_strs[np.random.randint(0, len(display_strs))]
        if loop_counter % (phase_number * period) == 0:
            c = np.random.choice([0, 1])
            bg = [0xFF80FF, 0x8080FF][c]
        # print(loop_counter)
        if current_phase == 0:
            clean_display()
            display.setColor(0xFFFF00)
            display.drawText(s, 10, 4)
        elif current_phase == 1:
            clean_display()
            display.setColor(0x00FFFF)
            display.drawText(s, 10, 4)
        elif current_phase == 2:
            clean_display()
            display.setColor(0xFF00FF)
            display.drawText(s, 10, 4)
    elif current_state == 'wait':
        motor.setVelocity(0)
    else:
        motor.setVelocity(0)
        
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    loop_counter += 1

# Enter here exit cleanup code.
