import math
import threading
import time
import numpy as np

from robot import VisionRobot

SPEED_AT_1 = 206

INSTRUCTION_SET = ["goals", "movements"]
USE_GOALS = 0
USE_MOVEMENTS = 1

X = 0
Y = 1

class Agent:

    movements: list # movement structure: []
    goal_position: list

    instruction_set: int

    position: list # [x, y] in mm
    phi: float # direction; -pi < phi < pi

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

    def start(self):
        self.robot.start()

    def add_movement(self, time=-1, dphi=0, radius=-1):
        if dphi == 0 and radius == -1:
            self.robot.setSpeed([1.0, 1.0])
            time.sleep(time)
            self.position[X] += np.sin(self.phi) * time * SPEED_AT_1
            self.position[Y] += np.cos(self.phi) * time * SPEED_AT_1
        elif dphi == 0 and time == -1:
            dtime = radius/206
            self.robot.setSpeed([1.0, 1.0])
        

    def _update_position(self):
        ...
        
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