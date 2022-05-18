"""dance_robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from collections.abc import Iterable
from controller import Robot
import numpy as np

pos = [[0,1],[0,0],[1,0],[0.5,-1],[-0.5,-1],[-1,0]]

class DanceRobot:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.loop_counter = 0
        self.motors = []
        for i in range(1, 5):
            self.motors.append(self.robot.getDevice('wheel' + str(i)))
            self.motors[-1].setPosition(np.inf)
    
        self.wheel_radius = 0.04
        self.basic_speed_w = 4.0
        self.basic_speed_l = self.basic_speed_w*self.wheel_radius
    
    def set_wheel_velocity(self, v):
        if not isinstance(v, Iterable):
            for m in self.motors:
                m.setVelocity(v)
        elif len(v) == 2:
            self.motors[0].setVelocity(v[0])
            self.motors[2].setVelocity(v[0])
            self.motors[1].setVelocity(v[1])
            self.motors[3].setVelocity(v[1])
        elif len(v) == 4:
            for m, v_ in zip(self.motors, v):
                m.setVelocity(v_)
        else:
            print('Invalid vecocity for motors')
            return 1
        
    def move_to_destination(self):
        des_pos = pos[self.id]
        
        desired_dis = 1.5
        desired_heading = np.pi/3
        
        left_wheel_speed = 2.0
        right_wheel_speed = -2.0

        velocity = self.wheel_radius*left_wheel_speed
        arc = self.wheel_radius*abs(right_wheel_speed
                            - left_wheel_speed)
        time_needed = desired_heading/(arc/0.185)
        time_for_dis = (5/4)*desired_dis/velocity
        
        if self.loop_counter < int(time_needed*1000/self.timestep):
            left_wheel_speed = 2.0
            right_wheel_speed = -2.0
        
        elif self.loop_counter < int(time_for_dis*1000/self.timestep):
            left_wheel_speed = 2.0
            right_wheel_speed = 2.0
        else:
            left_wheel_speed = 0
            right_wheel_speed = 0

        self.set_wheel_velocity([left_wheel_speed, right_wheel_speed])
    
    def dance_circle(self):
        delay = self.id * ((5/4)*0.3/(self.basic_speed_l))
        if self.loop_counter <= int(delay*1000/self.timestep):
            self.set_wheel_velocity(self.basic_speed_w)
        else:
            self.set_wheel_velocity([self.basic_speed_w*0.8,
                                     self.basic_speed_w*1.2])
    
    def step(self):
        if self.robot.step(self.timestep) != -1:
            self.id = int(self.robot.getCustomData())
            self.dance_circle()
            self.loop_counter += 1
        else:
            self.robot.cleanup()
            return 1

if __name__ == "__main__":
    robot = DanceRobot()
    while True:
        robot.step()
