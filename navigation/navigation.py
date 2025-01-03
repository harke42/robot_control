import math
import threading
import time
import numpy as np

import robot.visionrobot as vsrob
VELOCITY_AT_1 = 206

X = 0
Y = 1

DIST_BETWEEN_MOTORS = 108 # [mm]
TIME_SCALOR = 4/3
RADIUS_SCALOR = 4/5

class Navigation:

    movements: list # movement structure: [dphi, radius, time]

    robot_set_speed: function

    update_thread: threading.Thread
    queue_lock: threading.Lock

    def __init__(self, set_speed_func):
        self.movements = []
        self.robot_set_speed = set_speed_func
        self.queue_lock = threading.Lock()
        self.update_thread = threading.Thread(target=self._movement_task)

    def start(self):
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
        
        s_l = self._speed_to_motor_input(v_l)
        s_r = self._speed_to_motor_input(v_r)
        self.robot_set_speed([s_l, s_r])
        
        if vtime > dtime:
            time.sleep(dtime)
            self.robot_set_speed([0.0, 0.0])
            time.sleep(vtime-dtime)
        else:
            time.sleep(vtime)
            if not len(self.movements) > 0:
                self.robot_set_speed([0.0, 0.0])