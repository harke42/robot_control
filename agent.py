import navigation.navigation as nav
import robot.visionrobot as visrob
import aruco_detection.aruco_detector as arc

class Agent:
    
    robot:          visrob.VisionRobot
    navigation:     nav.Navigation
    aruco_detector: arc.ArucoDetector

    def __init__(self):
        self.robot = visrob.VisionRobot()
        
        self.navigation = nav.Navigation(set_speed_func=self.robot.setSpeed)

        self.robot.registerExternalWifiCallback(identifier='add_movement', callback=self.navigation.add_movement, 
                                                arguments=['dphi', 'radius', 'vtime'], description='Add Movement to movement queue of robot')
        self.robot.registerExternalWifiCallback(identifier='add_movements', callback=self.navigation.add_movements,
                                                arguments=['list'], description='Add multiple movements to Queue at once')
        
        self.aruco_detector = arc.ArucoDetector()

    def start(self):
        self.robot.start()
        self.navigation.start()
        self.aruco_detector.start()