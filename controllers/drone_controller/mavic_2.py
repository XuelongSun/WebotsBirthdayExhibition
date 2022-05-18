import numpy as np
from webots import WebotsRunner

class Mavic2Pro(WebotsRunner):
    def __init__(self, name):
        super(Mavic2Pro, self).__init__(name)
        
        self.motors = self.get_actuator(["front left propeller",
                                         "front right propeller",
                                         "rear left propeller",
                                         "rear right propeller"])
        for k, v in self.motors.items():
            v.setPosition(np.inf)
            v.setVelocity(0.0)
        
        self.cameras = self.get_sensor(['camera'])
        self.camera_motors = self.get_actuator(['camera roll', 'camera pitch', 'camera yaw'])
        for k, v in self.camera_motors.items():
            v.setPosition(0)

        self.camera_pos_sensors = self.get_sensor(['camera pitch sensor',
                                                   'camera roll sensor',
                                                   'camera yaw sensor'])
        self.camera_angle_limit = {'pitch':[-0.5, 1.7],
                                   'roll': [-0.5, 0.5],
                                   'yaw':[-1.7, 1.7]}
        
        self.sensors = self.get_sensor(['inertial unit', 'gps', 'gyro', 'compass'])
        
        self.leds = self.get_actuator(['front left led', 'front right led'])
        # control constant
        self.k_vertical_thrust = 68.5;  # with this thrust, the drone lifts.
        self.k_vertical_offset = 0.6;   # Vertical offset where the robot actually targets to stabilize itself.
        self.k_vertical_p = 3.0;        # P constant of the vertical PID.
        self.k_roll_p = 50.0;           # P constant of the roll PID.
        self.k_pitch_p = 30.0;          # P constant of the pitch PID.
        self.target_altitude = 1.0;
        
        self.roll_disturbance = 0.0
        self.pitch_disturbance = 0.0
        self.yaw_disturbance = 0.0
        
        self.state = 'wait'
        
        # wait one second
        self.wait(1000)
        
        print(self.get_camera_states())
    
    def get_camera_states(self):
        angles = dict.fromkeys([s.split(' ')[1] for s in self.camera_pos_sensors.keys()])
        for k, v in self.camera_pos_sensors.items():
            angles[k.split(' ')[1]] = v.getValue()
        return angles
    
    def get_camera_state(self, name='pitch'):
        return self.camera_pos_sensors['camera {} sensor'.format(name)].getValue()
        
    def set_camera_state(self, name='pitch', angle=0):
        if angle <= self.camera_angle_limit[name][0]:
            angle = self.camera_angle_limit[name][0]
        if angle >= self.camera_angle_limit[name][1]:
            angle = self.camera_angle_limit[name][0]
        self.camera_motors['camera {}'.format(name)].setPosition(angle)
        
    def keyboard_process(self):
        key = 0
        key = self.keyboard.getKey()
        self.roll_disturbance = 0.0
        self.pitch_disturbance = 0.0
        self.yaw_disturbance = 0.0
        if key == ord("W"):
            self.pitch_disturbance = 2.0;
        elif key == ord("S"):
            self.pitch_disturbance = -2.0
        elif key == self.keyboard.LEFT:
            self.yaw_disturbance = -1.3
        elif key == self.keyboard.RIGHT:
            self.yaw_disturbance = 1.3
        elif key == self.keyboard.UP:
            self.target_altitude += 0.01
            print("target altitude: %f [m]\n", self.target_altitude);
        elif key == self.keyboard.DOWN:
            self.target_altitude -= 0.01
            print("target altitude: %f [m]\n", self.target_altitude);
        elif key == ord("A"):
            self.roll_disturbance = 1.0
        elif key == ord("D"):
            self.roll_disturbance = -1.0
        elif key == self.keyboard.SHIFT + self.keyboard.UP:
            p = self.get_camera_state('pitch')
            self.set_camera_state('pitch', p + 0.02)
        elif key == self.keyboard.SHIFT + self.keyboard.DOWN:
            p = self.get_camera_state('pitch')
            self.set_camera_state('pitch', p - 0.02)
        elif key == self.keyboard.SHIFT + self.keyboard.LEFT:
            p = self.get_camera_state('roll')
            self.set_camera_state('roll', p + 0.02)
        elif key == self.keyboard.SHIFT + self.keyboard.RIGHT:
            p = self.get_camera_state('roll')
            self.set_camera_state('roll', p - 0.02)
        elif key == ord(" "):
            if self.state == 'wait':
                self.state = 'hover'
                print('drone started...')
            # elif self.state == 'hover':
            #     self.state = 'wait'
        elif key == ord("F"):
            self.robot.setCustomData('finish')

    
    def run(self):
        if self.state == 'hover':
            # states
            roll = self.sensors['inertial unit'].getRollPitchYaw()[0] + np.pi/2.0
            pitch = self.sensors['inertial unit'].getRollPitchYaw()[1] 
            altitude = self.sensors['gps'].getValues()[1]
            roll_acc = self.sensors['gyro'].getValues()[0]
            pitch_acc = self.sensors['gyro'].getValues()[1]
            
            # flash led
            led_s = self.loop_count % 10
            self.leds['front left led'].set(led_s)
            self.leds['front right led'].set(1 - led_s)
            
            # stablize camera
            # self.camera_motors['camera roll'].setPosition(-0.115*roll_acc)
            # self.camera_motors['camera pitch'].setPosition(-0.1*pitch_acc)

            # PID control
            roll_input = self.k_roll_p * np.clip(roll, -1.0, 1.0) + roll_acc + self.roll_disturbance
            pitch_input = self.k_pitch_p * np.clip(pitch, -1.0, 1.0) - pitch_acc + self.pitch_disturbance
            yaw_input = self.yaw_disturbance
            difference_altitude = np.clip(self.target_altitude - altitude + self.k_vertical_offset, -1.0, 1.0)
            vertical_input = self.k_vertical_p * difference_altitude ** 3.0
            # motor output
            v = self.k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
            self.motors['front left propeller'].setVelocity(v)
            v = self.k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input;
            self.motors['front right propeller'].setVelocity(-v)
            v = self.k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input;
            self.motors['rear left propeller'].setVelocity(-v)
            v = self.k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input;
            self.motors['rear right propeller'].setVelocity(v)
        else:
            led_s = self.loop_count % 30
            self.leds['front left led'].set(led_s)
            self.leds['front right led'].set(1 - led_s)