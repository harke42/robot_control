import math
import threading
import time
import numpy as np

from robot import VisionRobot

VELOCITY_AT_1 = 206

INSTRUCTION_SET = ["goals", "movements"]
USE_GOALS = 0
USE_MOVEMENTS = 1

X = 0
Y = 1

DIST_BETWEEN_MOTORS = 108 # [mm]

class Agent:

    movements: list # movement structure: [time, dphi, radius]
    goal_position: list

    instruction_set: int

    position: list # [x, y] in [mm]
    phi: float # direction in [rad]; -pi < phi < pi

    robot: VisionRobot

    #controller parameters
    k_phi: int      #proportional phi coefficient
    k_v: int        #proportional v coefficient

    update_thread: threading.Thread

    def __init__(self, start_pos=[0.0,0.0], start_dir=0.0, instruction_set=0, robot=None):
        self.position = start_pos
        self.goal_position = [0.0, 0.0]
        self.movements = []
        self.instruction_set = instruction_set
        self.direction = start_dir
        self.robot = robot
        if instruction_set == USE_MOVEMENTS:
            self.update_thread = threading.Thread(target=self._movement_task)

    def start(self):
        self.robot.start()
        self.update_thread.start()

    def add_movement(self, time=-1, dphi=-1, radius=-1):
        self.movements.append([time, dphi, radius])

    def _movement_task(self):
        while True:
            try:
                if len(self.movements) > 0:
                    movement = self.movements.pop(0)
                    self._calculate_movement(time=movement[0], dphi=movement[1], radius=movement[2])

            except Exception as e:
                print("Error in Movement Task!")
                print(e)
                break

    #TBD: _goal_task()

    def _speed_to_motor_input(self, velocity):
        return velocity/VELOCITY_AT_1

    def _vr_vl_from_radius(self, radius):
        ratio_vr_vl = (2*radius + DIST_BETWEEN_MOTORS)/(2*radius - DIST_BETWEEN_MOTORS)
        v_l = 0
        v_r = 0
        if ratio_vr_vl > 0:
            # case v_r larger than v_l --> left turn
            v_r = VELOCITY_AT_1
            v_l = v_r/ratio_vr_vl
        
        else:
            # case v_l larger than v_r --> right turn
            v_l = VELOCITY_AT_1
            v_r = ratio_vr_vl * v_l
            
        return v_l, v_r


    def _calculate_movement(self, time=-1, dphi=-1, radius=-1):
        """ time in [s], dphi in [rad], radius in [mm] """

        if INSTRUCTION_SET != USE_MOVEMENTS:
            print("Tried to add movement while in GOAL mode")
            return
        v_l = 0
        v_r = 0
        
        if dphi == 0 and radius == -1:
            v_l = 1.0
            v_r = 1.0

        elif dphi == 0 and time == -1:
            time = radius/206
            v_l = 1.0
            v_r = 1.0

        elif time == -1 and radius != -1:
            v_l, v_r = self._vr_vl_from_radius(radius)
            time = (dphi*DIST_BETWEEN_MOTORS)/(v_r - v_l)
        
        elif dphi == -1 and radius != -1:
            v_l, v_r = self._vr_vl_from_radius(radius)
            dphi = (v_r - v_l)*time * 1/DIST_BETWEEN_MOTORS
            
        else:
            print("Uncompleted movement command, skipping!")
            print("Input: time"+str(time)+" dphi=" + str(dphi) + " radius=" + str(radius))
            return
        
        s_l = self._speed_to_motor_input(v_l)
        s_r = self._speed_to_motor_input(v_r)
        self.robot.setSpeed([s_l, s_r])
        time.sleep(time)
        self._update_position(dphi, v_l, v_r)

    def _update_position(self, dphi, v_l, v_r):
        self.position[X] += 0.5*(v_r + v_l)*(np.sin(self.phi + dphi) - np.sin(self.phi))
        self.position[X] += 0.5*(v_r + v_l)*(-np.cos(self.phi + dphi) + np.cos(self.phi))
        self.phi += dphi

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