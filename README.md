# ucsd_robo_car_simple_ros

<img src="ucsd_ros_logo.png">

### A simple ROS package using OpenCV on a 1/10 RC car chassis with ackerman steering that can detect and track road lines or lanes in a driver-less mode. 

<div>

## Table of Contents

  - [**Dependencies**](#dependencies)
    - [cv2](#cv2)
    - [adafruit_servokit](#adafruit_servokit)
    - [cv_bridge](#cv_bridge)
  - [**Environment Configuration**](#environment-configuration)
    - [openCV Setup](#1-opencv-setup)
    - [Virtual Environment Setup](#2-virtual-environment-setup)
    - [Upgrading](#3-upgrading)
    - [Install Adafruit Library](#4-install-adafruit-library)
    - [Install ROS Melodic](#5-install-ros-melodic)
    - [Access this repository](#6-access-this-repository)
    - [Enable X11 forwarding](#7-enable-x11-forwarding)
  - [**Work Flow To Use This Repository**](#work-flow-to-use-this-repository)
  - [**Nodes**](#nodes)
    - [throttle_client](#throttle_client)
    - [steering_client](#steering_client)
    - [camera_server](#camera_server)
    - [lane_detection_node](#lane_detection_node)
    - [lane_guidance_node](#lane_guidance_node)
    - [ros_racer_calibration_node](#ros_racer_calibration_node)
  - [**Topics**](#topics)
    - [steering](#steering)
    - [throttle](#throttle)
    - [camera_rgb](#camera_rgb)
    - [centroid](#centroid)
  - [**Launch**](#launch)
    - [throttle and steering launch](#throttle-and-steering-launch)
    - [lane detection launch](#lane-detection-launch)
    - [ros racer calibration launch](#ros-racer-calibration-launch)
    - [ros racer launch](#ros-racer-launch)
  - [**Tools**](#tools)
    - [ROS Guide Book](#ros-guide-book)
    - [Run Indvidual Programs](#run-indvidual-programs)
    - [Decoder](#decoder)
  - [**Troubleshooting**](#troubleshooting)
    - [Camera not working](#camera-not-working)
    - [Throttle and steering not working](#throttle-and-steering-not-working)
    - [Error With CV_Bridge Conversion From Image Message To OpenCV Image](#error-with-cv_bridge-conversion-from-image-message-to-opencv-image)
    - [ROS Version Is Not Compatible with Python3](#ros-version-is-not-compatible-with-python3)
  - [**Demonstration videos**](#demonstration-videos)
    - [Lane detection example with yellow filter](#lane-detection-example-with-yellow-filter)
    - [Blue color detection](#blue-color-detection)
    - [Yellow color detection and line width specification](#yellow-color-detection-and-line-width-specification)
    - [Throttle and steering](#throttle-and-steering)
    - [Manipulating image dimensions](#manipulating-image-dimensions)

<div align="center">

## Dependencies

</div>

### [cv2](https://opencv.org/)

OpenCV is a library, in our case for Python, that provides high-level functions for computer vision and image processing.

### [adafruit_servokit](https://circuitpython.readthedocs.io/projects/servokit/en/latest/)

Adafruit Servokit is a Python library that provides a high-level interface with low-level PWM controls. For this package, the library is used to control PWM servos and a ESC connected to channels of the PCA9685 I2C breakout board.
more details <a href="https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython" >here</a>


### [cv_bridge](http://wiki.ros.org/cv_bridge)

CV Bridge provides functions to easily convert (encode/decode) in between ROS image message types to OpenCV-workable Numpy arrays.

<div align="center">

## Environment Configuration

</div>

### **1. openCV Setup**

  **a. Check if you have openCV for python3**

   `python3`

   then enter

   `import cv2`

  **b. If no error occurs, you're good to go. Otherwise issue the command below for barebones version**

   `sudo apt-get install python3-pip`

   `pip3 install --upgrade pip`

   `pip3 install opencv-python`


  **c. Check again to see if opencv was compiled correctly for python3**

   `python3`

  then enter

   `import cv2`

**_No errors should have happened, if so, make sure you used pip3 and not pip when running the install command above_**

more details <a href="https://pypi.org/project/opencv-python/" >here</a>

_**if you want to compile from source follow steps below**_

  **d. (OPTIONAL) build instructions for openCV** <a href="https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html" >here</a>

### **2. Virtual Environment Setup**

  **a. Creat environment**

   `python3 -m pip install --user virtualenv`

   `sudo apt-get install python3-venv`

   `python3 -m venv --system-site-packages env`

   `source env/bin/activate`

   `python3 -m pip install requests`

  **b. Environment details**

   Get path to executable
   `which python`

   Get python version
   `python --version`

   List of packages
   `pip list`

   Site packages location
   `python -m site`

  **c. Add PYHTHONPATH**

   `nano ~/.bash_profile`

   Add this line to bash file

   `export PYTHONPATH="<path to virtual env>/lib/python3.6"`

  **d. Activate Environment (for new terminals)**

   `source env/bin/activate`

<div align="center">

   **NOTE**
   _**WHILE IN VIRTUAL ENVIRONMENT, DO NOT USE "sudo"  TO INSTALL PIP PACKAGES, THESE WILL INSTALL TO ROOT INSTEAD OF VIRTUAL ENVIRONMENT**_

   more details <a href="https://realpython.com/python-virtual-environments-a-primer/" >here</a>
   and <a href="https://bic-berkeley.github.io/psych-214-fall-2016/using_pythonpath.html" >here</a>

</div>


### **3. Upgrading**


`pip install pyyaml`

`pip install rospkg`

`pip install --upgrade pip`

`pip install --upgrade pyinstaller`


### **4. Install Adafruit Library**

`pip install adafruit-circuitpython-pca9685`

`pip install adafruit-circuitpython-servokit`

more details <a href="https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython" >here</a>

### **5. Install ROS Melodic**

Instructions found <a href="https://docs.google.com/document/d/1LxcTvSTRHVxSnv3x_cJ667loWgzCt7ikTJk51dKAFSs/edit?usp=sharing" >here</a>


### **6. Access this repository**

  **a. Generate an SSH key and provide it to Gitlab for access to repositories**

   `ssh-keygen # Use all defaults`

  **b. Then press enter until you get to an empty comand line, then**

   `cat $HOME/.ssh/id_rsa.pub`

  **c. Then copy the ssh key and go back to Gitlab. Click on your user profile at the top right corner of the screen then**
     
  **click on _preferences_ from the drop down menu. Now a new panel on the left hand side of the screen wil apear, click on _SSH Keys_,**
     
  **then paste your SSH key into the text field and submit it.**

  **d. Create ROS workspace and obtain copy of ucsd_robo_car_simple_ros repository**

   `mkdir projects && cd projects`

   `mkdir catkin_ws && cd catkin_ws`

   `mkdir src && cd src`
   
   `git clone git@gitlab.com:djnighti/ucsd_robo_car_simple_ros.git`

  **e. Build ucsd_robo_car_simple_ros package:**

   `cd ..`

   `catkin_make`

   `source devel/setup.bash`

   `rospack profile`

  **f. OPTIONALLY (RECOMMENDED) add some lines of code to the bash script so that every time a new terminal is opened, the virtual env is activated and this ROS package is compiled and sourced**

   `nano ~/.bashrc`

   add the following lines of code at the end of the bash script

   `cd`

   `source env/bin/activate`

   `cd projects/catkin_ws`

   `catkin_make`

   `source devel/setup.bash`

   Then press 
   
   **ctrl-x** 
   
   Then press 
   
   **y**  (yes) 
   
   and then press 
   
   **enter** 
   
   to save an quit
  
  **g. Now try this to make sure it was compiled correctly:**

   `roscd ucsd_robo_car_simple_ros`

  **h. Now give yourself permissions to access all files in repo:**

   `chmod -R 777 .`

  **i. (ONLY DO THIS AS NEEDED) Now as this remote repository is updated, enter the following commands to update the local repository on the jetson:**
   
   `roscd ucsd_robo_car_simple_ros`
   
   `git stash`

   `git pull`

   `chmod -R 777 .`

## **7. Enable X11 forwarding**


Associated file: **x11_forwarding_steps.txt**

Some jetsons may not have this enabled, so if needed please read the steps in this file to setup X11 forwarding

<div align="center">

## **Work Flow To Use This Repository**

</div>

1. **ALWAYS RUN ROSCORE IN A TERMINAL ON EVERY BOOT UP OF THE JETSON**

`roscore`

2. Calibrate the camera, throttle and steering values using the [ros_racer_calibration_node](#ros_racer_calibration_node)

`roslaunch ucsd_robo_car_simple_ros ros_racer_calibration_launch.launch`

3. Launch [ros racer launch](#ros racer launch)

`roslaunch ucsd_robo_car_simple_ros ros_racer_launch.launch`

4. Tune parameters in step 2 until desired behavior is achieved

<div align="center">

## Nodes

</div>

### **throttle_client**

Associated file: **throttle_client.py**

This node subscribes to the [**throttle**](#Topics) topic. We use subscriber callback function
to validate and normalize throttle value, and then use the [**adafruit_servokit**](#adafruit_servokit)
module on **channel 2** for sending signals to the hardware.

This node is also responsible for reading and setting the throttle calibration values.

### **steering_client**

Associated file: **steering_client.py**

Similar to [**throttle_client**](#throttle_client), this node subscribes to the [**steering**](#Topics)
topic and passes the signals to the hardware. The steering servo is on **channel 1**.

Plenty of information on how to use the adafruit_servokit libraries can be found <a href="https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython" >here</a> and <a href="https://github.com/adafruit/Adafruit_CircuitPython_ServoKit" >here</a> 

### **camera_server**

Associated file: **camera_server.py**

This node simply reads from the camera with cv2's interface and publishes the image to the
[**camera_rgb**](#Topics) topic. Before publishing, the image is reformatted from the cv image format
so it can be passed through the ROS topic message structure.

### **lane_detection_node**

Associated file: **lane_detection.py**

This node subscribes from the [**camera_rgb**](#Topics) topic and uses opencv to identify line
information from the image, and publish the information of the lines centroid to the [**centroid**](#centroid). 

The color scheme is defined as follows:

- 2 contours : green bounding boxes and a blue average centroid
- 1 contour : green bounding box with a single red centroid

Below show the image post processing techniques, cv2 methods and the logic applied respectively.

<div align="center">
  <img src="filtering_process.png">
  <img src="applying_methods.png">
  <img src="applying_logic.png">
</div>

### **lane_guidance_node**

Associated file: **lane_guidance.py**

This node subscribes to the centroid topic, calculates the throttle and steering
based on the centroid value, and then publish them to their corresponding topics.
Throttle is based on whether or not a centroid exists - car goes faster when centroid is present and slows down when there is none.
Steering is based on a proportional controller implemented by the calculating the error between the centroid found in [**lane_detection_node**](#lane_detection_node) and the heading of the car. 

Gains can be tweaked in the lane_guidance.py script.

### **ros_racer_calibration_node**

Associated file: **ros_racer_calibration_node.py**

Calibrate the camera, throttle and steering in this node by using the sliders to find:
- the right color filter 
- desired image dimmensions
- throttle values for both the optimal condtion (error = 0) and the non optimal condtion (error !=0) AKA go fast when error=0 and go slow if error !=0
- steering sensitivty change the Kp value to adjust the steering sensitivty (as Kp --> 1 steering more responsive, as Kp --> 0  steering less responsive) 

| Property   | Info |
| ----------  | --------------------- |
| lowH, highH | Setting low and high values for Hue  | 
| lowS, highS | Setting low and high values for Saturation | 
| lowV, highV | Setting low and high values for Value | 
| Inverted_filter | Specify to create an inverted color tracker | 
| min_width, max_width | Specify the width range of the line to be detected  | 
| number_of_lines | Specify the number of lines to be detected  | 
| error_threshold | Specify the acceptable error the robot will consider as approximately "no error" | 
| frame_width | Specify the width of image frame (horizontal cropping) | 
| rows_to_watch | Specify the number of rows (in pixels) to watch (vertical cropping) | 
| rows_offset | Specify the offset of the rows to watch (vertical pan) | 
| Steering_sensitivity | Specify the proportional gain of the steering | 
| Steering_value | Specify the steering value | 
| Throttle_mode | Toggle this slider at the end of calibration to the following 3 modes. |
| Throttle_mode 0 | zero_throttle_mode (find value where car does not move) 
| Throttle_mode 1 | zero_error_throttle_mode (find value for car to move when there is **no error** in steering)
| Throttle_mode 2 | error_throttle_mode(find value for car to move when there is **some error** in steering)| 
| Throttle_value | Specify the throttle value to be set in each of the throttle modes| 

More morphological transfromations and examples can be found <a href="https://docs.opencv.org/3.4/db/df6/tutorial_erosion_dilatation.html" >here</a> and <a href="https://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html" >here</a>

These values are saved automatically to a configuration file, so just press control-c when the deepracer is calibrated.

<div align="center">

## Topics

</div>

### **throttle** 
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| /throttle   | std_msgs.msg.Float32  | Float value from -1 to 1 for controlling throttle          |

#### **steering**
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| /steering   | std_msgs.msg.Float32  | Float value from -1 to 1 for controlling steering          |

#### **camera_rgb**
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| /camera_rgb | sensor_msgs.msg.Image | Image last read from USB camera image                      |

#### **centroid**
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| /centroid   | std_msgs.msg.Float32  | Float value for that represents the error of the x coordinate of centroid in camera image space|

<div align="center">

## Launch

</div>

#### **throttle and steering launch**

Associated file: **throttle_and_steering_launch.launch**

This file launches both [**throttle_client**](#throttle_client) and [**steering**](#Topics) seperately because these topics can take some time to initialize which can delay productivity. Launch this script once and use the other launch files listed below to get the robot moving.

`roslaunch ucsd_robo_car_simple_ros throttle_and_steering_launch.launch`

#### **lane Detection launch**

Associated file: **laneDetection_launch.launch**

This file will launch [**lane_detection_node**](#lane_detection_node), [**lane_guidance_node**](#lane_guidance_node), [**camera_server**](#camera_server) and load the color filter parameters created using [ros_racer_calibration_node](#ros_racer_calibration_node)

**Before launching, please calibrate the robot first while on the stand!**

`roslaunch ucsd_robo_car_simple_ros laneDetection_launch.launch`

#### **ros racer calibration launch**

Associated file: **ros_racer_calibration_launch.launch**

[ros_racer_calibration_node](#ros_racer_calibration_node)

This file will launch [**camera_server**](#camera_server), [ros_racer_calibration_node](#ros_racer_calibration_node) and [throttle and steering launch](#throttle-and-steering-launch)

`roslaunch ucsd_robo_car_simple_ros ros_racer_calibration_launch.launch`

#### **ros racer launch**
[ros racer launch](#ros racer launch)
This file will launch [throttle and steering launch](#throttle-and-steering-launch) and [lane detection launch](#lane-detection-launch)

`roslaunch ucsd_robo_car_simple_ros ros_racer_launch.launch`

<div align="center">

## Tools 

</div>

#### ROS Guide Book

For help with using ROS in the terminal and in console scripts, check out this google doc <a href="https://docs.google.com/document/d/1u7XS7B-Rl_emK3kVKEfc0MxHtwXGYHf5HfLlnX8Ydiw/edit?usp=sharing" >here</a> to see tables of ROS commands and plenty of examples of using ROS in console scripts.

#### **Run Indvidual Programs**

To run any indvidual program, enter this into the terminal and change file_name.py to whatever python file is in the repo

`rosrun ucsd_robo_car_simple_ros file_name.py`

#### **Decoder** 

Associated file: **decoder.py**

This provides a solution for cv_bridge not working and decodes the incoming image into a numpy array that is then passed to the [**camera_rgb**](#Topics) topic. If cv_bridge is built with python3, then this file is not neccessary.

<div align="center">

## Troubleshooting

</div>

#### **Camera not working** 

If while running [deepracer_calibration_node](#deepracer_calibration_node) or [aws_rosracer.launch](#aws_rosracerlaunch) and if the cv2 windows do not open, then follow the procedure below to potentially resolve the issue.

1. Make sure camera is plugged in all the way into its USB socket
1. See if image feed is coming through in another application like cheese. (Enter `cheese` into terminal window)
1. Check to see if the camera topic is publishing data `rostopic echo /camera_rgb`
1. Restart ROS core 
1. Reboot if none of the above worked and try again `sudo reboot now`

If the camera is still not working after trying the procedure above, then it could be a hardware issue. (Did the car crash?)

#### **Throttle and steering not working** 

If while running [ros_racer_calibration_node](#ros_racer_calibration_node) or [ros racer launch](#ros racer launch) and the throttle and steering are unresponsive, then follow the procedure below to potentially resolve the issue.

1. Make sure ESC is turned on
1. Make sure battery is plugged in
1. Make sure battery has a charge
1. Make sure servo and ESC wires are plugged into the pwm board into the correct channels correctly
1. Check to see if the steering and throttle topics are publishing data `rostopic echo /steering` and `rostopic echo /throttle`
1. Verify that the throttle values found in [ros_racer_calibration_node](#ros_racer_calibration_node) were loaded properly when running  [ros racer launch](#ros racer launch) (Values will be printed to the terminal first when running the launch file) 
1. Restart ROS core 
1. Reboot if none of the above worked and try again `sudo reboot now`

If the Throttle and steering are still not working after trying the procedure above, then it could be a hardware issue. (Did the car crash?)

#### **Error With CV_Bridge Conversion From Image Message To OpenCV Image**

Using **bridge_object.imgmsg_to_cv2()** threw errors on our Jetson Nano environment, so we had to resort to our own image decoder function. Function **decodeImage()** can be imported from **decoder.py**. If you don't want to use our function, the problem can be avoided by properly building CV_Bridge with Python3 in the ROS package.

An alternative solution can be found <a href="https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674" >here</a>

### **ROS Version Is Not Compatible with Python3**
If your're having issues using python3, then there is a chance that the virtual environment (explained in [**Environment Configuration**](#environment-configuration)) was not setup properly. Try setting up another environment to see if that solves the issue.

More info found 
<a href="https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674" >here</a>

<div align="center">

## **Demonstration videos** 

</div>

<div align="center">

#### Lane detection example with yellow filter

[![lane detection example with yellow filter](https://j.gifs.com/6WRqXN.gif)](https://youtu.be/f4VrncQ7HU)

</div>

<div align="center">

#### Number of lines to detect
[![Number of lines to detect](https://j.gifs.com/qQ7Lvk.gif)](https://youtu.be/5AVL68BTD0U)

</div>

<div align="center">

#### Error threshold
[![Error threshold](https://j.gifs.com/k28Xmv.gif)](https://youtu.be/ied1TDvpDK4)

</div>

<div align="center">

#### Blue color detection

[![Blue color detection](https://j.gifs.com/PjZoj6.gif)](https://youtu.be/c9rkRHGGea0)

</div>

<div align="center">

#### Yellow color detection and line width specification

[![Yellow color detection and line width specification](https://j.gifs.com/BrLJro.gif)](https://youtu.be/l2Ngtd461DY)

</div>

<div align="center">

#### Throttle and steering

[![Throttle and steering](https://j.gifs.com/362n6p.gif)](https://youtu.be/lhYzs5v6jtI)

</div>

<div align="center">

#### Manipulating image dimensions

[![Manipulating image dimensions](https://j.gifs.com/lR5oR1.gif)](https://youtu.be/_DhG6dduYPs)

</div>

