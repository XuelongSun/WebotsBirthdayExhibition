import sys
import os
from collections import deque
import time

import math 
import numpy as np
import cv2

from controller import Motion
from webots import WebotsRunner
# motion and gait module
libraryPath = os.path.join(os.environ.get("WEBOTS_HOME"), 'projects', 'robots',
                           'robotis', 'darwin-op', 'libraries', 'python39')
libraryPath = libraryPath.replace('/', os.sep)
sys.path.append(libraryPath)
from managers import RobotisOp2GaitManager, RobotisOp2MotionManager


class DarwinOP(WebotsRunner):
    def __init__(self, name):
        super(DarwinOP, self).__init__(name)

        self.motor_names = ('ShoulderR', 'ShoulderL', 'ArmUpperR', 'ArmUpperL',
                            'ArmLowerR', 'ArmLowerL', 'PelvYR', 'PelvYL',
                            'PelvR', 'PelvL', 'LegUpperR', 'LegUpperL',
                            'LegLowerR', 'LegLowerL', 'AnkleR', 'AnkleL',
                            'FootR', 'FootL', 'Neck', 'Head')
        self.motor_direction = {
            'ShoulderR': -1,
            'ShoulderL': 1,
            'Neck': 1,
            'Head': 1,
            'ArmUpperR': 1,
            'ArmUpperL': 1,
            'ArmLowerR': -1,
            'ArmLowerL': 1,
            'PelvYR': -1,
            'PelvYL': -1,
            'PelvR': -1,
            'PelvL': -1,
            'LegUpperR': 1,
            'LegUpperL': -1,
            'LegLowerR': 1,
            'LegLowerL': -1,
            'AnkleR': -1,
            'AnkleL': 1,
            'FootR': 1,
            'FootL': 1,
        }
        # sensors
        self.position_sensor_names = [n + 'S' for n in self.motor_names]
        self.inertial_sensor_names = ['Gyro', 'Accelerometer']

        # official gait and motion manager, we may use ourselves' gait algorithm
        self.motion_manager = RobotisOp2MotionManager(self.robot)
        self.gait_manager = RobotisOp2GaitManager(self.robot, "config.ini")
        self.gait_manager.setBalanceEnable(True)
        self.initial_position = None
        
        # related to fallen detection
        self.face_up_count = 0
        self.face_down_count = 0
        self.fallen_acc_threshold = 80
        self.fallen_acc_steps = 20
        
        self.key_history = deque([], maxlen=10)
        
    def play_user_motion_sync(self, motion):
        motion.play()
        while not motion.isOver():
            self.my_step()
        return 0
    
    def initialize(self):
        print('Initializing robot...')
        self.position_sensors = self.get_sensor(self.position_sensor_names)
        self.inertial_sensors = self.get_sensor(self.inertial_sensor_names)
        self.camera = self.get_sensor(['Camera'])['Camera']
        self.camera_height = self.camera.getHeight()
        self.camera_width = self.camera.getWidth()
        self.camera_fov = self.camera.getFov()
        self.motors = self.get_actuator(self.motor_names)
        self.my_step()
        
        # stand up
        self.motion_manager.playPage(1)
        self.wait(200)
        # get position sensors values
        self.initial_position = self.get_position_sensor_values()
        self.loop_count = 0
        
        print('Successfully initialized.')
        
        self.is_walking = False
        
        return self.sensors, self.actuators

    def move_to_destination(self):
        m = self.robot.getCustomData().split(',')
        if 'motion' in m:
            self.gait_manager.stop()
            self.motion_manager.playPage(1)
            self.wait(500)
            self.motion_manager.playPage(int(m[1]))
            self.wait(1000)
            self.gait_manager.start()
            # self.gait_manager.setAAmplitude(float(m[1]))
            # self.gait_manager.setXAmplitude(0)
        else:
            c_pos = [float(x) for x in m[0].split(' ')]
            d_pos = [float(x) for x in m[2].split(' ')]
            h = float(m[1])
            # calculate direction
            dir = np.arctan2(d_pos[0] - c_pos[0], d_pos[1] - c_pos[1])
            diff = (h - dir + np.pi) % (np.pi*2)-np.pi
            if diff > 0:
            # a = np.clip(0.1*(), -1.0, 1.0)
                self.gait_manager.setAAmplitude(-0.5)
                self.gait_manager.setXAmplitude(0.5)
            else:
                self.gait_manager.setAAmplitude(0.5)
                self.gait_manager.setXAmplitude(0.5)
    
    def set_motor_position(self, motor_name, value, offset=0):
        # direction
        value *= self.motor_direction[motor_name]
        # offset, can get from position sensors of a specific gesture
        value += offset
        # clip
        min_p = self.motors[motor_name].getMinPosition()
        max_P = self.motors[motor_name].getMaxPosition()
        if value < min_p:
            value = min_p
        if value > max_P:
            value = max_P
        # set motor position
        self.motors[motor_name].setPosition(value)

        return value
    
    def get_position_sensor_values(self):
        position = dict.fromkeys(self.position_sensor_names)
        for k, v in self.position_sensors.items():
            position[k] = v.getValue()
        return position
    
    def set_head_yaw(self, angle=0):
        # -1.81rad(-100deg, right) to 1.81rad(100deg left)
        v = self.set_motor_position('Neck', np.deg2rad(angle))
        return v
    
    def set_head_pitch(self, angle=0):
        # -0.36rad(-20deg, down) to 0.96rad(55deg up)
        # <-0.2rad(<-11deg, feet will be seen)
        v = self.set_motor_position('Head', np.deg2rad(angle))
        return v
    
    def keyboard_process(self):
        key = 0
        key = self.keyboard.getKey()
        if key != -1:
            self.key_history.append(key)
            # print(self.key_history)
        ## motion
        if key == ord("W"):
            self.gait_manager.setXAmplitude(1.0)
        elif key == ord("S"):
            self.gait_manager.setXAmplitude(-1.0)
        elif key == ord("A"):
            self.gait_manager.setAAmplitude(0.5)
        elif key == ord("D"):
            self.gait_manager.setAAmplitude(-0.5)
        elif key == ord(" "):
            if not self.is_walking:
                self.gait_manager.start()
                self.is_walking = True
            else:
                self.gait_manager.stop()
                self.is_walking = False
        ## heading
        elif key == self.keyboard.UP:
            # head up
            motor_position = self.get_position_sensor_values()
            head_position = np.rad2deg(motor_position['HeadS']) + 2
            self.set_head_pitch(head_position)
        elif key == self.keyboard.LEFT:
            # head left
            motor_position = self.get_position_sensor_values()
            neck_position = np.rad2deg(motor_position['NeckS']) + 2
            self.set_head_yaw(neck_position)
        elif key == self.keyboard.DOWN:
            # head down
            motor_position = self.get_position_sensor_values()
            head_position = np.rad2deg(motor_position['HeadS']) - 2
            self.set_head_pitch(head_position)
        elif key == self.keyboard.RIGHT:
            # head right
            motor_position = self.get_position_sensor_values()
            neck_position = np.rad2deg(motor_position['NeckS']) - 2
            self.set_head_yaw(neck_position)
        elif key == ord("F"):
            self.robot.setCustomData('finish')

        return key
    
    def get_img(self, hsv=False):
        # get current image
        img = np.array(self.camera.getImageArray(), dtype=np.uint8)
        if hsv:
            # convert to HSV for color recognition
            img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        return img

    def save_img(self, filename='img.jpg', img=None):
        try:
            if img is None:
                self.camera.saveImage(filename, quality=50)
            else:
                cv2.imwrite(filename, img)
            return True
        except:
            return False
    
    def check_if_fallen(self):
        acc = self.inertial_sensors['Accelerometer'].getValues()
        if acc[1] < 512.0 - self.fallen_acc_threshold:
            self.face_up_count += 1
        else:
            self.face_up_count = 0

        if acc[1] > 512.0 + self.fallen_acc_threshold:
            self.face_down_count += 1
        else:
            self.face_down_count = 0
        
        # if fallen, gets up
        if self.face_up_count > self.fallen_acc_steps:
            return 'Fallen face up'
        elif self.face_down_count > self.fallen_acc_steps:
            return 'Fallen face down'
        else:
            return None
    
    def ready_to_walk(self):
        self.motion_manager.playPage(9)
        self.wait(100)
        # liftup the head
        self.set_head_pitch(40)

    def walk_step(self):
        self.gait_manager.step(self.time_step)
        self.gait_manager.setXAmplitude(0.0)
        self.gait_manager.setAAmplitude(0.0)
        return True

