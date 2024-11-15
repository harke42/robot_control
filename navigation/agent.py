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

    def __init__(self, start_pos=[0.0,0.0], start_dir=0.0, instruction_set=1):
        self.position = start_pos
        self.goal_position = [0.0, 0.0]
        self.phi = start_dir
        self.movements = []
        self.instruction_set = instruction_set
        self.direction = start_dir
        self.robot = vsrob.VisionRobot()
        self.robot.registerExternalWifiCallback(identifier='add_movement', callback=self.add_movement, 
                                                arguments=['dphi', 'radius', 'vtime'], description='Add Movement to movement queue of robot')
        if instruction_set == USE_MOVEMENTS:
            self.update_thread = threading.Thread(target=self._movement_task)

    def start(self):
        self.robot.start()
        self.update_thread.start()

    def add_movement(self, dphi=0, radius=-1, vtime=-1):
        if self.instruction_set != USE_MOVEMENTS:
            print("Tried to add movement while in GOAL mode")
            return
        self.movements.append([dphi, radius, vtime])

    def _movement_task(self):
        while True:
            try:
                if len(self.movements) > 0:
                    movement = self.movements.pop(0)
                    if len(movement) == 2:
                        self._calculate_movement(dphi=movement[0], radius=movement[1])
                    else:
                        self._calculate_movement(dphi=movement[0], radius=movement[1], vtime=movement[2])

            except Exception as e:
                print("Error in Movement Task!")
                print(e)

    #TBD: _goal_task()

    def _speed_to_motor_input(self, velocity):
        return velocity/VELOCITY_AT_1

    def _vr_vl_from_radius(self, radius):
        ratio_vr_vl = (RADIUS_SCALOR*2*radius + DIST_BETWEEN_MOTORS)/(RADIUS_SCALOR*2*radius - DIST_BETWEEN_MOTORS)
        v_0 = VELOCITY_AT_1
        v_1 = v_0/ratio_vr_vl
        
        return v_1, v_0


    def _calculate_movement(self, dphi=0, radius=-1, vtime=-1):
        """ time in [s], dphi in [rad], radius in [mm] """

        v_l = 0
        v_r = 0
        
        if dphi == 0:
            if radius == 0:
                # stop
                pass
            elif radius > 0:
                v_l = VELOCITY_AT_1
                v_r = VELOCITY_AT_1
                vtime = radius/VELOCITY_AT_1
            else:
                raise Exception("Invalid Combination of dphi and radius: dphi = 0 and radius < 0")
            
        else:
            if dphi > 0:
                v_l, v_r = self._vr_vl_from_radius(radius)
            else:
                v_r, v_l = self._vr_vl_from_radius(radius)
            
            vtime = TIME_SCALOR * (dphi*DIST_BETWEEN_MOTORS)/(v_r - v_l)

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
        time.sleep(vtime)
        self._update_position(radius, dphi)
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

"""
SAMPLE_TIME = 0.01

class Agent:
    state: [float, float, float]    #x, y, psi
    input: [float, float]           #v, psi_dot
    goal_positions: list[list[float, float]]

    robot: VisionRobot
    _thread: threading.Thread

    # Controller Constants
    v_max = 0.25
    v_min = 0.1
    psi_dot_max = math.pi
    radius = 0.01

    # Regler Koeffizienten
    Kp_psi = 10
    Kd_psi = 1
    Ki_psi = 0

    # letzte Fehlerwerte
    last_error_dist = 0
    last_error_psi = 0

    def __init__(self, robot):
        self.state = [0, 0, 0]
        self.input = [0, 0]
        self.goal_positions = [[0.0, 0.0]]
        self.robot = robot

        self._thread = threading.Thread(target=self._threadFunction)

    def start(self):
        self._thread.start()
        print("Agent started")

    def setInput(self, input):
        self.input = input

    def addGoal(self, x, y):
        self.goal_positions.append([x, y])

    def _control(self):
        # PD-Control for Psi
        error_psi = np.arctan2((self.goal_position[1] - self.state[1]), (self.goal_position[0] - self.state[0])) - \
                    self.state[2]
        if error_psi > math.pi:
            error_psi -= 2 * math.pi
        if error_psi < -math.pi:
            error_psi += 2 * math.pi
        print(f"Error Psi: {error_psi}")
        # Proportional
        psi_dot_p = self.Kp_psi * error_psi
        # Integral
        psi_dot_i = self.Ki_psi * ((error_psi + self.last_error_psi) / SAMPLE_TIME)
        # Differential
        psi_dot_d = self.Kd_psi * ((error_psi - self.last_error_psi) / SAMPLE_TIME)
        self.last_error_psi = error_psi
        # Stellgröße
        psi_dot = psi_dot_d + psi_dot_p + psi_dot_i

        error_s = np.sqrt((self.goal_position[0]-self.state[0])**2 + (self.goal_position[1]-self.state[1])**2)

        print(f"Error Distance: {error_s}")

        # Tank turn until error_psi < 5°
        if np.abs(error_psi) > np.radians(5):
            v = 0
        else:
            v = self.v_max
            psi_dot = 0

        if error_s < 0.1:
            v = 0

        self.setInput([v, psi_dot])

    def _threadFunction(self):
        while True:
            goal_position = self.goal_positions[0]
            if np.abs(self.state[0] - goal_position[0]) > 0.01 or np.abs(self.state[1] - goal_position[1]) > 0.01:
                error_psi = np.arctan2((goal_position[1] - self.state[1]), (goal_position[0] - self.state[0])) - \
                            self.state[2]
                self.robot.turn(error_psi)
                time.sleep(np.abs(error_psi))
                self.state[2] += error_psi
                error_s = np.sqrt((goal_position[0] - self.state[0]) ** 2 + (goal_position[1] - self.state[1]) ** 2)
                self.robot.setSpeed([0.5, 0.5])
                time.sleep(np.abs(error_s))
                self.state[0] = goal_position[0]
                self.state[1] = goal_position[1]
                self.robot.setSpeed([0, 0])
                print(f"Robot at {self.state[0]}|{self.state[1]}")
            elif len(self.goal_positions) > 1:
                self.goal_positions.pop(0)
            time.sleep(0.1)


if __name__ == '__main__':
    from utils.stm32.stm32_flash.reset import reset as stm32_reset
    from robot.VisionRobot.VisionRobot import VisionRobot

    stm32_reset(0.25)
    robot = VisionRobot()
    robot.init()
    robot.start()
    agent = Agent(robot)
    agent.start()
"""