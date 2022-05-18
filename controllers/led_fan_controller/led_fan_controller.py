"""led_fan_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from time import time
from controller import Robot
import numpy as np
import struct
import cv2

# dis_image = cv2.imread('flash_bg1.png', cv2.IMREAD_UNCHANGED)
# interval_len = dis_image.shape[0]/14

# create the Robot instance.
robot = Robot()

def rgb888(c):
    cb = b'\x00' + c
    return struct.unpack('>L', cb)[0]

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

leds = []
for i in range(8):
    leds.append(robot.getDevice('led' + str(i)))

motor = robot.getDevice('motor')

motor.setPosition(np.inf)


# led.enable(timestep)

colors = [rgb888(b'\xff\x00\x00'),
          rgb888(b'\x00\xff\x00'),
          rgb888(b'\x00\x00\xff'),
          rgb888(b'\x00\xff\xff'),]

# initializing
counter = 0
for led in leds:
    led.set(0)

period = 12
motor.setVelocity(-np.pi*2/(period*timestep)*1000)
# motor.setVelocity(1000)
# motor.setVelocity(10)

while robot.step(timestep) != -1:
    # basic pattern - 
    p = counter%period
    for i, led in enumerate(leds):
        if (i == p) or (i == p%7):
            led.set(colors[1])
        else:
            led.set(0)
    
    # display a picture
    # alpha = 2*np.pi/period * (counter%period) - np.pi/2
    # for i, led in enumerate(leds):
    #     x = i*interval_len*np.cos(alpha)
    #     y = i*interval_len*np.sin(alpha)
    #     color = dis_image[int(x), int(y)]
    #     color_byte = rgb888(struct.pack('B', color[0]) + 
    #                         struct.pack('B', color[1]) + 
    #                         struct.pack('B', color[2]))
    #     # transfrt
    #     led.set(color_byte)
    

    # random color flashing
    # if counter % 30 == 0:
    #     for led in leds:
    #         turn_off = np.random.choice([1, -1])
    #         if turn_off == 1:
    #             color = np.random.randint(0, 255, size=3)
    #             color_byte = rgb888(struct.pack('B', color[0]) + 
    #                                 struct.pack('B', color[1]) + 
    #                                 struct.pack('B', color[2]))
    #         else:
    #             color_byte = 0    
    #         led.set(color_byte)

    counter += 1
    
robot.cleanup()
# Enter here exit cleanup code.
