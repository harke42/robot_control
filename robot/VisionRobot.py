import ctypes
import threading
import time
#import robot.VisionRobot.aruco_detection.aruco_detector as arcd

from board.board import RobotControl_Board
from robot.communication.twipr_communication import TWIPR_Communication
from utils.ctypes_utils import struct_to_dict

''' Define Structs START '''


class motor_input_struct(ctypes.Structure):
    _fields_ = [("input_left", ctypes.c_float), ("input_right", ctypes.c_float)]


class motor_speed_struct(ctypes.Structure):
    _fields_ = [("input_left", ctypes.c_float), ("input_right", ctypes.c_float)]


class CommunicationData(ctypes.Structure):
    _fields_ = [
        ("tick", ctypes.c_uint32),
        ("debug", ctypes.c_uint8),
        ("state", ctypes.c_uint8),
        ("battery_voltage", ctypes.c_float),

        ("goal_speed_l", ctypes.c_float),
        ("goal_speed_r", ctypes.c_float),
        ("rpm_l", ctypes.c_float),
        ("rpm_r", ctypes.c_float),
        ("velocity_l", ctypes.c_float),
        ("velocity_r", ctypes.c_float),
        ("velocity_forward", ctypes.c_float),
        ("velocity_turn", ctypes.c_float),

        ("imu_gyr_x", ctypes.c_float),
        ("imu_gyr_y", ctypes.c_float),
        ("imu_gyr_z", ctypes.c_float),
        ("imu_acc_x", ctypes.c_float),
        (
            "imu_acc_y", ctypes.c_float),
        ("imu_acc_z", ctypes.c_float),
    ]


''' Define Structs END '''


class VisionRobot:
    #aruco_detector: arcd.ArucoDetector
    board: RobotControl_Board
    communication: TWIPR_Communication
    _thread: threading.Thread
    data: CommunicationData

    def __init__(self, camera_version="v3", image_server_ip="localhost"):
        #self.aruco_detector = arcd.ArucoDetector(version=camera_version,
        #                                         stream_if=image_server_ip)

        self.board = RobotControl_Board(device_class='robot',
                                        device_type='visionrobot',
                                        device_revision='v2',
                                        device_id='visionrobot1',
                                        device_name='Vision Robot 1')

        self.communication = TWIPR_Communication(board=self.board)
        self.communication.serial.interface.registerCallback('rx_stream', self._rx_callback)

        self.communication.wifi.addCommand(identifier='setSpeed',
                                           callback=self.setSpeed,
                                           arguments=['speed'],
                                           description='Set the speed of the motors')

        self.communication.wifi.addCommand(identifier='turn',
                                           callback=self.turn,
                                           arguments=['phi'],
                                           description='Turn robot by phi [rad]')

        self.communication.wifi.addCommand(identifier='goTo',
                                           callback=self.goTo,
                                           arguments=['x', 'y'],
                                           description='Go to specified x,y position in local coordinate system')

        self._thread = threading.Thread(target=self._threadFunction)

        self.data = CommunicationData()

        self.board.init()
        self.communication.init()

    # === METHODS ======================================================================================================

    # ------------------------------------------------------------------------------------------------------------------
    def start(self):
        #self.aruco_detector.start()
        self.board.start()
        self.communication.start()
        self._thread.start()
        print("START VISION ROBOT ...")

    # ------------------------------------------------------------------------------------------------------------------

    '''Debug Function to check UART functionality
        Turns on and off LED Below USB C Port
    '''

    def debug(self, state):
        self.communication.serial.executeFunction(module=0x01,
                                                  address=0x01,
                                                  data=state,
                                                  input_type=ctypes.c_uint8,
                                                  output_type=None)

    def setSpeed(self, speed):
        assert (isinstance(speed, list))
        print(f"Set Speed to {speed}")
        input_struct = motor_input_struct(input_left=speed[0], input_right=speed[1])
        self.communication.serial.executeFunction(module=0x01,
                                                  address=0x02,
                                                  data=input_struct,
                                                  input_type=motor_input_struct)

#    def turn(self, phi):
#        print(f"Turn by {phi}")
#        cphi = ctypes.c_float(phi)
#        self.communication.serial.executeFunction(module=0x01, address=0x03, data=cphi, input_type=ctypes.c_float)

    # === PRIVATE METHODS ==============================================================================================
    ''' Send Stream to Hardware Manager '''

    def _threadFunction(self):
        while True:
            data = {'test': 1}
            self.communication.wifi.sendStream(data)
            time.sleep(0.1)

    ''' get Data from MC Firmware '''

    def _rx_callback(self, msg_data, *args, **kwargs):
        data = CommunicationData.from_buffer_copy(msg_data)
        self.data = struct_to_dict(data)
        # print(f"{data}")
