import numpy as np

from controller import Supervisor

def calculate_distance(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[2] - p2[2])**2)**0.5


class WorldController:
    def __init__(self):
        self.supervisor = Supervisor()
        self.robot = self.supervisor.getFromDef("Explorer")
        self.robot.getField("customData").setSFString("none")
        self.viewpoint = self.supervisor.getFromDef("VIEWPOINT")
        self.final_view = False
        self.view_counter = 1600
        self.tv1 = self.supervisor.getFromDef("TV1")
        self.tv1.getField("customData").setSFString('v1.mp4 wait')
        self.tv2 = self.supervisor.getFromDef("TV2")
        self.tv2.getField("customData").setSFString('v2.mov wait')
        
        self.cylinder_display_l = self.supervisor.getFromDef("CylinderDisplayL")
        self.cylinder_display_r = self.supervisor.getFromDef("CylinderDisplayR")
        self.cylinder_display_l.getField("customData").setSFString('wait')
        self.cylinder_display_r.getField("customData").setSFString('wait')
        
        self.balloon_light = [self.supervisor.getFromDef("BalloonLight1"),
                              self.supervisor.getFromDef("BalloonLight2")]
        for l in self.balloon_light:
            l.getField("spotLightIntensity").setSFFloat(0.0)
            
        self.cake = self.supervisor.getFromDef("CAKE_CANDEL")
        self.cake.getField("rotation").setSFRotation([0, 1, 0, 0])
        self.table_lamp = self.supervisor.getFromDef("DinningTable.TableLamp.Light")
        self.table_lamp.getField("on").setSFBool(False)
        self.ap = self.supervisor.getFromDef("DinningTable.TableLamp.Obj.LampShade.Appearance")
        self.ap.getField("emissiveIntensity").setSFFloat(0)
        self.desk_lamp = self.supervisor.getFromDef("DESK.lamp.Light")
        self.desk_lamp.getField("on").setSFBool(False)
        self.timestep = int(self.supervisor.getBasicTimeStep())
        self.loop_counter = 0
        
        self.waylights = []
        self.waylights_position = []
        # get all the way lights and turn it off
        i = 0
        while True:
            ln = 'WayLight' + str(i)
            l = self.supervisor.getFromDef(ln)
            if l is None:
                break
            self.waylights.append(l)
            p = l.getField("translation").getSFVec3f()
            self.waylights_position.append([p[0]*0.2, p[1]*0.2, p[2]*0.2])
            l.getField("pointLightIntensity").setSFFloat(1)
            l.getField('pointLightColor').setSFColor([1, 1, 1])
            i += 1
        
        self.tablelights_turned_on = False
        self.tablelights = []
        for i in range(1,10):
            ln = 'TableLight' + str(i)
            l = self.supervisor.getFromDef(ln + '.Light')
            self.tablelights.append(l)
            l.getField("on").setSFBool(True)
            
        self.helicopters = []
        self.helicopters_lights = []
        self.helicopter_positions = []
        self.helicopter_height = 2.5
        self.helicopter_state = ['wait']*4
        for i in range(4):
            n = 'HELICOPTER' + str(i)
            h = self.supervisor.getFromDef(n)
            if h is not None:
                h.getField("customData").setSFString(self.helicopter_state[i])
                self.helicopters.append(h)
                self.helicopter_positions.append(h.getField("translation").getSFVec3f())
                l = self.supervisor.getFromDef(n + '.Light')
                self.helicopters_lights.append(l)
                l.getField("on").setSFBool(False)
            else:
                print('Cannot Find Helicopter: {}'.format(n))
        
        self.parking_lights = []
        for i in range(4):
            n = 'PARKING' + str(i) + '.LIGHT'
            h = self.supervisor.getFromDef(n)
            if h is not None:
                self.parking_lights.append(h)
                h.getField("on").setSFBool(False)
        
        self.door_lights = [self.supervisor.getFromDef("DoorLight1"),
                            self.supervisor.getFromDef("DoorLight2")]
        for l in self.door_lights:
            l.getField("spotLightIntensity").setSFFloat(2.0)
        
        # video recorded
        self.wait_robot = False
        # viewpoints:
        # defined in format [[x,y,z],[x,y,z,alpha],follow or not]
        self.viewpoints = [[[-17.3, 0.705, 7.31], [0.21, 0.97, 0.12, 5.65], True], #o
                           [[-16.5, 0.532, 4.72], [-0.14, -0.98, -0.113, 1.21], True], #o
                           [[-15, 0.61, 1.13], [-0.02, 1, 0.06, 2.42], True], #o
                           [[-16.6, 0.49, 0.19], [-0.1, -1, -0.125, 1.32], True], #o
                           [[-14.4, 0.59, -0.9], [-0.17, 0.98, 0.113, 1.19], True],#o
                           [[-16.6, 0.805, -0.36], [0.37, 0.93, 0.06, 5.88], True],#o
                           [[-15.9, 0.548, -3.41], [-0.06, -1, -0.1, 1.99], True],#o
                           [[-13.9, 0.5, -3.5], [0, -1, -0.04, 3], True],#o
                           [[-8.76, 0.86, 0.2], [0.1, -1, -0.1, 4.7], False],#o
                           [[-11.6, 0.53, 0.75], [-0.97, -0.23, -0.07, 0.214], True],#o
                           [[-9.68, 1.41, -1.15], [-0.013, -0.97, -0.24, 2.98], False],#o-10
                           [[-8.8, 2.02, -0.2], [-0.01, -0.9, -0.4, 3.07], True],#o
                           [[-11.2, 0.95, 1.66], [-0.08, -1, -0.08, 1.53], True],#o
                           [[-6.63, 0.5, 1.07], [-0.83, 0.56, 0.04, 0.268], True],#o
                           [[-7.75, 0.76, -0.39], [-0.03, -1, -0.04, 1.62], True],#o
                           [[-4.64, 2.72, -0.02], [-0.3, -0.912, -0.3, 1.67], True],  # 15-balloon arc
                           [[-3.4, 1.17, 0], [-0.1, -1, -0.1, 1.58], True],#o
                           [[-1.16, 1.72, 0.16], [-0.18, -0.97, -0.18, 1.58], True],#o
                           [[1.74, 1.64, 2.17], [-0.85, 0.53, 0.07, 0.352], True],#o
                           [[-0.05, 0.73, -1.76], [-0.71, -0.695, -0.12, 0.446], True],#o
                           [[1.78, 1.57, -3.37], [-0.145, -1, -0.2, 1.93], True],#o-20
                           [[0, 1.23, -1.2], [-0.03, -1, -0.05, 1.69], True],#o
                           [[6.24, 1.06, 0.579], [-0.54, -0.8, -0.18, 0.704], False], # o 22-dinner table
                           [[7, 0.88, -1.55], [-0.36, -0.926, -0.114, 0.56], True], #o
                           [[14.3, 2.19, -5.45], [0, 0.97, 0.256, 3.16], True], # o habit area
                           [[14.3, 1.78, -1.8], [-0.04, 0.95, 0.31, 2.94], True], # 25-desk
                           [[13.1, 1.85, 3.76], [0.94, -0.34, -0.07, 5.75], True], # 26-table
                           [[9.95, 2.19, 2.56], [-0.64, -0.74, -0.18, 0.71], True], # 27-table
                           [[12.1, 2.28, -2.68], [-0.23, -0.935, -0.27, 1.76], True], # o
                           [[14.7, 0.612, -2.84], [-0.05, -1, -0.06, 1.27], True], # o
                           [[16.9, 1.05, -4.32], [0.1, 1, 0.13, 4.42], True], # o
                           [[18.4, 0.913, 2.88], [0, 0.985, 0.174, 3.05], True], # o 
                           [[2.23, 1.1, 2.72], [0, 0.985, 0.174, 3.08], True], #o 32-wishes
                           [[4, 1.94, 3.52], [-0.235, 0.955, 0.197, 1.4], False], #o 
                           ]
        # defined in format [[x, z]]
        # same length as viewpoints
        self.robot_waypoints = [[-15.5, 4.27],
                                [-15.4, 2.35],
                                [-15.8, 0],
                                [-15.6, -1.5],
                                [-15.8, -2.5],
                                [-14.5, -2.7],  # 5
                                [-13.8, -2.42],
                                [-13.6, -0.5],
                                [-11.6, -0.4],
                                [-9.5, -0.2],
                                [-8.5, 1],  # 10
                                [-8.9, 1.75],
                                [-7, 0.25],
                                [-6.13, -0.37],
                                [-2.6, 0],
                                [-0.8, 0],  # 15-balloon arc
                                [0.7, 0],
                                [1.75, 0],  # 17-helicopter and cake
                                [-0.2, -3],
                                [4, -3.5],
                                [4.89, -0.7],
                                [7, -0.5],  # 21-display
                                [7.5, -3.5],  # 
                                [14, -3.5],  # 
                                [14.2, -0.6],  # 
                                [13.6, 2],  # 25-desk
                                [11.2, 1.26],  # 26-table
                                [14, -1.26],  # 27-table
                                [16.2, -3.5],  # 
                                [18.5, -3.5],  # 
                                [18.5, 4.3],  # 30
                                [4, 4.0],  # 
                                [1, 4.0],  # 32-wishes
                                [0.5, 2.5],  # 33
                                ]
        # turn command
        self.special_motion_step = {17: 24}
        self.current_record_index = 0

    def update_way_lights(self, r_p):
        # calculate the distance between all the way lights and the robot
        for wl, p in zip(self.waylights, self.waylights_position):
            dis = calculate_distance(p, r_p)
            if dis > 2.0:
                ld = 0
            elif dis > 1.0:
                ld = 1.0/dis
            else:
                ld = 2.0
            pre_ld = wl.getField("pointLightIntensity").getSFFloat()
            if pre_ld < 0.5:
                wl.getField("pointLightIntensity").setSFFloat(ld)
        # balloon light
        if r_p[0] > -4.5:
            for l in self.balloon_light:
                l.getField("spotLightIntensity").setSFFloat(3)
        
        # dinning table light
        if r_p[0] > 7:
            self.table_lamp.getField("on").setSFBool(True)
            self.ap.getField("emissiveIntensity").setSFFloat(1.0)
        
        # dest light
        if r_p[0] > 8:
            self.desk_lamp.getField("on").setSFBool(True)
            
    def update_table_lights(self, r_p):
        if (r_p[0] > -10) and (r_p[2] > 0.3):
            # turn on table lights
            for wl in self.tablelights:
                wl.getField("on").setSFBool(True)
                self.tablelights_turned_on = True
    
    def control_helicopter(self, r_p):     
        for i, h in enumerate(self.helicopters):
            if self.helicopter_state[i] == 'wait':
                if r_p[0] > 1.0:
                    print('helicopter start!')
                    self.helicopter_state[i] = 'start'
                    h.getField('customData').setSFString(self.helicopter_state[i])
                    # turn on parking light
                    for l in self.parking_lights:
                        l.getField("on").setSFBool(True)
            elif self.helicopter_state[i] == 'start':
                pos = h.getField("translation").getSFVec3f()
                if pos[1] >= self.helicopter_height:
                    self.helicopter_state[i] = 'hover'
                    # turn on the light of the helicopter
                    self.helicopters_lights[i].getField("on").setSFBool(True)
                    h.getField('customData').setSFString(self.helicopter_state[i])
                    h.getField("translation").setSFVec3f([self.helicopter_positions[i][0],
                                                          self.helicopter_height, 
                                                          self.helicopter_positions[i][2]])
                # else:
                #     h.getField("translation").setSFVec3f([pos[0], pos[1], pos[2]])
            elif self.helicopter_state[i] == 'hover':
                # pass
                h.getField("translation").setSFVec3f([self.helicopter_positions[i][0],
                                        self.helicopter_height, 
                                        self.helicopter_positions[i][2]])
                # rotate the cake
                rot = self.cake.getField("rotation").getSFRotation()
                self.cake.getField("rotation").setSFRotation([0, 1, 0, rot[3] + 0.01])
    
    def move_view(self):
        if self.final_view_start_count <= self.view_counter:
            target_p = self.viewpoint.getField('position').getSFVec3f()
            for i in range(3):
                target_p[i] += self.view_p_interval[i]
            self.viewpoint.getField('position').setSFVec3f(target_p)
            return 0
        else:
            return 1
    
    def control_display_robot(self, r_p):
        tv1_x = self.tv1.getField("translation").getSFVec3f()[0]
        if tv1_x >= 9.5:
            self.tv1.getField("customData").setSFString('v1.mp4 play 0 0')
        elif r_p[0] > 5.0:
            self.tv1.getField("customData").setSFString('v1.mp4 play 10 2')
        else:
            self.tv1.getField("customData").setSFString('v1.mp4 wait')
            
        tv2_x = self.tv1.getField("translation").getSFVec3f()[0]
        if tv2_x >= 9.5:
            self.tv2.getField("customData").setSFString('v2.mov play 0 0')
        elif r_p[0] > 5.0:
            self.tv2.getField("customData").setSFString('v2.mov play -10 -3')
        else:
            self.tv2.getField("customData").setSFString('v2.mov wait')
    
    def control_cylinder_display(self, r_p):
        if r_p[0] > -1.5:
            self.cylinder_display_l.getField("customData").setSFString('play')
            self.cylinder_display_r.getField("customData").setSFString('play')
    
    def normal_running(self):
        if self.robot.getField("customData").getSFString() == 'finish':
            # turn off lights
            for wl in self.tablelights:
                wl.getField("on").setSFBool(False)
            self.table_lamp.getField("on").setSFBool(False)
            self.ap.getField("emissiveIntensity").setSFFloat(0.0)
            self.desk_lamp.getField("on").setSFBool(False)
            for l in self.balloon_light:
                l.getField("spotLightIntensity").setSFFloat(0)
            for l in self.helicopters_lights:
                l.getField("on").setSFBool(False)
            for l in self.door_lights:
                l.getField("spotLightIntensity").setSFFloat(0)
            # turn on waylights
            for l in self.waylights:
                n = int(l.getDef().split('t')[1])
                if (n >= 24) and (n <= 37):
                    l.getField('pointLightColor').setSFColor([1, 0.6, 1])
                l.getField("pointLightIntensity").setSFFloat(1.0)
            # landing helicopter
            for h in self.helicopters:
                h.getField("customData").setSFString("landing")
                
            # self.final_view = True
            # self.final_view_start_count = 0
            # # c_view_p = self.viewpoint.getField('position').getSFVec3f()
            # c_view_r = self.viewpoint.getField('orientation').getSFRotation()
            # k = np.array(c_view_r[:3])
            # theta = c_view_r[3]
            # # Rodrigues' rotation formula
            # # v = k/np.max(np.abs(k)) * (150/self.view_counter)
            # v = np.array([0, 0, 1])*(150/self.view_counter)
            # self.view_p_interval = v*np.cos(theta) + np.cross(k, v)*np.sin(theta) + k*(np.dot(k, v))*(1 - np.cos(theta))
            # self.view_p_interval = (150 - c_view[1])/self.view_counter
            # self.viewpoint.getField('orientation').setSFRotation([-0.5,0.5,0.5,2.03])
            # c_view = self.viewpoint.getField('position').getSFVec3f()
            # for i in range(3):
            #     self.view_p_interval[i] = (self.view_p_interval[i] - c_view[i])/self.view_counter
            # self.view_r_interval = [-0.6, 0.5, 0.6, 2.22]
            # c_view = self.viewpoint.getField('orientation').getSFRotation()
            # for i in range(4):
            #     self.view_r_interval[i] = (self.view_r_interval[i] - c_view[i])/self.view_counter
        else:
            r_p = self.robot.getField("translation").getSFVec3f()
            # self.control_helicopter()
            if self.loop_counter % 20 == 0:
                self.control_display_robot(r_p)
                self.control_cylinder_display(r_p)
                self.update_way_lights(r_p)
                if not self.tablelights_turned_on:
                    self.update_table_lights(r_p)
            if self.loop_counter % 4 == 0:
                self.control_helicopter(r_p)
    
    def video_recording(self):
        if self.current_record_index < len(self.viewpoints):
            r_p = self.robot.getField("translation").getSFVec3f()
            r_h = self.robot.getField("rotation").getSFRotation()[3]
            des_p = self.robot_waypoints[self.current_record_index]
            s = '{} {},{},{} {}'.format(r_p[0], r_p[2], r_h,
                                        des_p[0], des_p[1])
            self.robot.getField("customData").setSFString(s)
            if not self.wait_robot:
                view = self.viewpoints[self.current_record_index]
                self.viewpoint.getField('orientation').setSFRotation(view[1])
                self.viewpoint.getField('position').setSFVec3f(view[0])
                if view[2]:
                    self.viewpoint.getField('follow').setSFString('Explorer')
                else:
                    self.viewpoint.getField('follow').setSFString('')
                self.wait_robot = True
            if np.sqrt((r_p[0] - des_p[0])**2 + (r_p[2] - des_p[1])**2) <= 0.1:
                if self.current_record_index in self.special_motion_step.keys():
                    self.robot.getField("customData").setSFString('motion,{}'.format(
                        self.special_motion_step[self.current_record_index]
                    ))
                    # wait for robot turning
                    self.wait(10000)
                print('record:', self.current_record_index)
                self.current_record_index += 1
                self.wait_robot = False
                # if self.current_record_index >= len(self.viewpoints):
                #     print('video recorded done!')
                #     self.supervisor.simulationSetMode(
                #         self.supervisor.SIMULATION_MODE_PAUSE)
            
            if self.loop_counter % 20 == 0:
                self.control_display_robot(r_p)
                self.control_cylinder_display(r_p)
                # self.update_way_lights(r_p)
                if not self.tablelights_turned_on:
                    pass
                    # self.update_table_lights(r_p)
            if self.loop_counter % 4 == 0:
                self.control_helicopter(r_p)
        else:
            # finish and final view
            if self.final_view:
                if self.move_view():
                    self.supervisor.simulationSetMode(
                        self.supervisor.SIMULATION_MODE_PAUSE)
                else:
                    self.final_view_start_count += 1
            else:
                self.final_view = True
                self.final_view_start_count = 0
                self.viewpoint.getField('position').setSFVec3f([1.08, 3.24, 2.35])
                c_view_r = [-0.54, 0.65, 0.536, 2]
                self.viewpoint.getField('orientation').setSFRotation(c_view_r)
                k = np.array(c_view_r[:3])
                theta = c_view_r[3]
                # Rodrigues' rotation formula
                v = np.array([0, 0, 1])*(150/self.view_counter)
                self.view_p_interval = v*np.cos(theta) + np.cross(k, v)*np.sin(theta) + k*(np.dot(k, v))*(1 - np.cos(theta))
                self.wait(1000)
                # turn off lights
                for wl in self.tablelights:
                    wl.getField("on").setSFBool(False)
                self.table_lamp.getField("on").setSFBool(False)
                self.ap.getField("emissiveIntensity").setSFFloat(0.0)
                self.desk_lamp.getField("on").setSFBool(False)
                for l in self.balloon_light:
                    l.getField("spotLightIntensity").setSFFloat(0)
                for l in self.helicopters_lights:
                    l.getField("on").setSFBool(False)
                for l in self.door_lights:
                    l.getField("spotLightIntensity").setSFFloat(0)
                # turn on waylights
                for l in self.waylights:
                    n = int(l.getDef().split('t')[1])
                    if (n >= 24) and (n <= 37):
                        l.getField('pointLightColor').setSFColor([1, 0.6, 1])
                    l.getField("pointLightIntensity").setSFFloat(1.0)
                # landing helicopter
                for h in self.helicopters:
                    h.getField("customData").setSFString("landing")
                
                self.wait(3000)
            # final view points
            
        
    def step(self):
        if self.supervisor.step(self.timestep) != -1:
            # normal running
            # self.normal_running()
            # recording
            self.video_recording()
            self.loop_counter += 1
            return 0
        else:
            return 1
    
    def wait(self, ms):
        start_time = self.supervisor.getTime()
        s = ms / 1000.0
        while s + start_time >= self.supervisor.getTime():
            self.supervisor.step(self.timestep)

if __name__ == "__main__":
    clt = WorldController()
    print('Exploration Starts!')
    clt.wait(32)
    while True:
        clt.step()
            
        
    