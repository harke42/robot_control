import math
import threading
import time
import numpy as np

import robot.visionrobot as vsrob
VELOCITY_AT_1 = 206

INSTRUCTION_SET = ["goals", "movements"]
USE_GOALS = 0
USE_MOVEMENTS = 1

X = 0
Y = 1

DIST_BETWEEN_MOTORS = 108 # [mm]
TIME_SCALOR = 4/3
RADIUS_SCALOR = 4/5

class Agent:

    movements: list # movement structure: [dphi, radius, time]
    goal_position: list

    instruction_set: int

    position: list # [x, y] in [mm]
    phi: float # direction in [rad]; -pi < phi < pi

    robot: vsrob.VisionRobot

    #controller parameters
    k_phi: int      #proportional phi coefficient
    k_v: int        #proportional v coefficient

    update_thread: threading.Thread
    queue_lock: threading.Lock

    def __init__(self, start_pos=[0.0,0.0], start_dir=0.0, instruction_set=1):
        self.position = start_pos
        self.goal_position = [0.0, 0.0]
        self.phi = start_dir
        self.movements = []
        self.instruction_set = instruction_set
        self.direction = start_dir
        self.robot = vsrob.VisionRobot()
        self.queue_lock = threading.Lock()
        self.robot.registerExternalWifiCallback(identifier='add_movement', callback=self.add_movement, 
                                                arguments=['dphi', 'radius', 'vtime'], description='Add Movement to movement queue of robot')
        self.robot.registerExternalWifiCallback(identifier='add_movements', callback=self.add_movements,
                                                arguments=['list'], description='Add multiple movements to Queue at once')
        if instruction_set == USE_MOVEMENTS:
            self.update_thread = threading.Thread(target=self._movement_task)

    def start(self):
        self.robot.start()
        self.update_thread.start()

    def add_movements(self, list):
        for element in list:
            if len(element) == 2:
                self.add_movement(dphi=element[0], radius=element[1])
            elif len(element) == 3:
                self.add_movement(dphi=element[0], radius=element[1], vtime=element[2])
            else:
                continue

    def add_movement(self, dphi=0, radius=-1, vtime=-1):
        if self.instruction_set != USE_MOVEMENTS:
            print("Tried to add movement while in GOAL mode")
            return
        self.queue_lock.acquire()
        self.movements.append([dphi, radius, vtime])
        self.queue_lock.release()

    def _movement_task(self):
        while True:
            try:
                if len(self.movements) > 0:
                    self.queue_lock.acquire()
                    movement = self.movements.pop(0)
                    self.queue_lock.release()
                    if len(movement) == 2:
                        self._calculate_movement(dphi=movement[0], radius=movement[1])
                    else:
                        self._calculate_movement(dphi=movement[0], radius=movement[1], vtime=movement[2])

            except Exception as e:
                print("Error in Movement Task!")
                print(e)
            time.sleep(1)

    #TBD: _goal_task()

    def _speed_to_motor_input(self, velocity):
        return velocity/VELOCITY_AT_1

    def _vr_vl_from_radius(self, radius):
        ratio_vr_vl = (RADIUS_SCALOR*2*radius + DIST_BETWEEN_MOTORS)/(RADIUS_SCALOR*2*radius - DIST_BETWEEN_MOTORS)
        v_0 = VELOCITY_AT_1
        v_1 = v_0/ratio_vr_vl
        
        return v_1, v_0


    def _calculate_movement(self, dphi=0, radius=-1, vtime=0):
        """ time in [s], dphi in [rad], radius in [mm] """

        v_l = 0
        v_r = 0
        dtime = 0
        
        if dphi == 0:
            if radius == 0:
                # stop
                pass
            elif radius > 0:
                v_l = VELOCITY_AT_1
                v_r = VELOCITY_AT_1
                dtime = radius/VELOCITY_AT_1
            else:
                raise Exception("Invalid Combination of dphi and radius: dphi = 0 and radius < 0")
            
        else:
            if dphi > 0:
                v_l, v_r = self._vr_vl_from_radius(radius)
            else:
                v_r, v_l = self._vr_vl_from_radius(radius)
            
            dtime = TIME_SCALOR * (dphi*DIST_BETWEEN_MOTORS)/(v_r - v_l)

        ##if dphi == 0 and radius == -1:
        ##    v_l = VELOCITY_AT_1
        ##    v_r = VELOCITY_AT_1
##
        ##elif dphi == 0 and vtime == -1:
        ##    vtime = radius/VELOCITY_AT_1
        ##    if radius > 0:
        ##        v_l = VELOCITY_AT_1
        ##        v_r = VELOCITY_AT_1
##
        ##elif vtime == -1 and radius != -1:
        ##    if dphi > 0:
        ##        v_l, v_r = self._vr_vl_from_radius(radius)
        ##        
        ##    else:
        ##        v_r, v_l = self._vr_vl_from_radius(radius)
        ##    
        ##    vtime = TIME_SCALOR * (dphi*DIST_BETWEEN_MOTORS)/(v_r - v_l)
        ##
        ##elif dphi == 0 and radius != -1:
        ##    v_l, v_r = self._vr_vl_from_radius(radius)
        ##    dphi = (v_r - v_l)*vtime * 1/DIST_BETWEEN_MOTORS
            
        ##else:
        ##    print("Uncompleted movement command, skipping!")
        ##    print("Input: time"+str(vtime)+" dphi=" + str(dphi) + " radius=" + str(radius))
        ##    return
        
        s_l = self._speed_to_motor_input(v_l)
        s_r = self._speed_to_motor_input(v_r)
        self.robot.setSpeed([s_l, s_r])
        time.sleep(dtime)
        self._update_position(radius, dphi)

        if vtime > dtime:
            self.robot.setSpeed([0.0, 0.0])
            time.sleep(vtime-dtime)
        else:
            if not len(self.movements) > 0:
                self.robot.setSpeed([0.0, 0.0])


    def _update_position(self, dphi, radius):
        if dphi > 0:
            phi_M_0 = self.phi - (np.pi/2)
            phi_M_1 = phi_M_0 + dphi
        else: 
            phi_M_0 = self.phi + (np.pi/2)
            phi_M_1 = phi_M_0 + dphi

        if phi_M_0 > np.pi:
            phi_M_0 -= 2*np.pi
        elif phi_M_0 <= -np.pi:
            phi_M_0 += 2*np.pi
        if phi_M_1 > np.pi:
            phi_M_1 -= 2*np.pi
        elif phi_M_1 <= -np.pi:
            phi_M_1 += 2*np.pi
        delta_x = radius * (np.cos(phi_M_1) - np.cos(phi_M_0))
        delta_y = radius * (np.sin(phi_M_1) - np.sin(phi_M_0))
        self.position[X] += delta_x
        self.position[Y] += delta_y