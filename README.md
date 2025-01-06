# Robot Control :robot:

The use of this project is to control a VisionRobot, more precise a robot, running on a Raspberry Pi like platform. 

Features included are navigation planning, a camera integration of a Picamera to detect and interpret cv2 Aruco Markers and a communcation stack to communicate with a host PC and the Micro Controller.

---

## <u>Quick Start</u>

### Tested Hard- and Software

- Raspi: Compute Module 4

- Camera: [Raspberry Pi Camera Module 3](https://www.berrybase.de/raspberry-pi-camera-module-3-wide-12mp)

- OS: Raspbian Bullseye, 64 Bit

### Startup

Setup a Raspi/Compute-Module-Robot 

- Checkout the [STM32 microcontroller code](https://git.tu-berlin.de/theses1/bsc_cooperative_sensing) and install it

- Connect to your robot via ethernet or wifi

- Make sure you can ping/ssh on to your robot

- Keep going with Option [a)](#installation-with-install-scripts-[linux]) if you are on Linux or skip to Option [b)](#installation-with-install-scripts-[windows]) for Windows, or with option [c)]() if you want to do everything by hand and understand, what is going on

### Installation with install scripts [Linux]

- change to *install* folder of the repo 

- run *install.sh* script with the login name and hostname/ip of your robot
  
  ```bash
  cd install && ./install.sh username@hostname/hostip 
  ```

- you will have to authenticate for your raspi two times
  
  1. to copy the necessary files 
  
  2. to run the actual install script on the raspi itself

. go to [verification](#verification-of-installation)

### Installation with install scripts [Windows]

- copy the *install_robot_control.sh*, *requirements.txt* and *test_aruco.py* to your raspi

- run the *install_robot_control.sh* script (sudo authentication might be necessary)
  
  ```bash
  ./install_robot_control.sh
  ```

- go to [last step](#verification-of-installation)

### Installation by hand

- install following debian packages:
  
  ```bash
  libgl1 libcap-dev python3 python3-pip python3-picamera2  python3-dev python3-libcamera python3-kms++
  ```

- install pip packages defined in *install/requirements.txt*
  
  ```bash
  pip3 install -r requirements.txt
  ```

- install adafruit-blinka
  
  ```bash
  wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/raspi-blinka.py
  sudo pip install adafruit-python-shell
  sudo python3 raspi-blinka.py
  ```
  
  follow raspi-blinka installation guide, don't reboot yet on finish

- add `dtoverlay=imx708,cam1` to */boot/config.txt* 
  
  parameter overview by camera type:
  
  | Camera Version                           | Overlay Command       |
  | ---------------------------------------- | --------------------- |
  | v3                                       | dtoverlay=imx708,cam1 |
  | v2                                       | dtoverlay=imx219,cam1 |
  | v1 (and most of<br/>non raspi officials) | dtoverlay=ov5647,cam1 |

- go to [verification](#verification-of-installation)

### Verification of Installation

- reboot your raspi

- check with the `test_aruco.py` if the installation was succesfull

- on succes, the script returns something like
  
  ```
  Initializing Camera
  Testing camera
  Found Marker  [3]  in distance  0.4829704748794956
  ```
  
  if you have some Aruco Markers in your camera sight or 
  
  ```
  Initializing Camera
  Testing camera
  No Markers found, but everything else is working!
  ```
  
  if you have no markers in sight of the camera

### Possible Problems

- Camera Problems:
  
  - Cable connected correctly?
  
  - `dtoverlay=imx708,cam1` in */boot/config.txt*?
  
  - read [here](https://www.raspberrypi.com/documentation/computers/camera_software.html#configuration) for more information

- ...


