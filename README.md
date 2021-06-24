# ucsd_robo_car_simple_ros

A simple ROS package using OpenCV on a 1/10 RC car chassis with ackerman steering that can follow a yellow line or stay between two white lines.

<div>

## Table of Contents

  - [**Dependencies**](#dependencies)
    - [cv2](#cv2)
    - [adafruit_servokit](#adafruit_servokit)
    - [cv_bridge](#cv_bridge)
  - [**Environment Configuration**](#environment-configuration)
  - [**Work Flow To Use This Repository**](#work-flow-to-use-this-repository)
  - [**Nodes**](#nodes)
    - [throttle_client](#throttle_client)
    - [steering_client](#steering_client)
    - [camera_server](#camera_server)
    - [line_detection_node](#line_detection_node)
    - [lane_detection_node](#lane_detection_node)
    - [lane_guidance_node](#lane_guidance_node)
    - [camera_values_node](#camera_values_node)
  - [**Topics**](#topics)
    - [steering](#steering)
    - [throttle](#throttle)
    - [camera_rgb](#camera_rgb)
    - [centroid](#centroid)
  - [**Launch**](#launch)
    - [throttle and steering launch](#throttle-and-steering-launch)
    - [line detection launch](#line-detection-launch)
    - [lane detection launch](#lane-detection-launch)
    - [ucsd_robo_car_calibration_launch](#ucsd_robo_car-calibration-launch)
  - [**Tools**](#tools)
    - [Run Indvidual Programs](#run-indvidual-programs)
    - [Throttle and Steering Calibration](#throttle-and-steering-calibration)
    - [Find Camera Parameters](#find-camera-parameters)
    - [Decoder](#decoder)
  - [**Issues and Fixes**](#issues-and-fixes)
    - [Error With CV_Bridge Conversion From Image Message To OpenCV Image](#error-with-cv_bridge-conversion-from-image-message-to-opencv-image)
    - [Throttle Not working](#throttle-not-working)
    - [ROS Version Is Not Compatible with Python3](#ros-version-is-not-compatible-with-python3)
## Dependencies

### [cv2](https://opencv.org/)

OpenCV is a library, in our case for Python, that provides high-level functions for computer vision and image processing.

### [adafruit_servokit](https://circuitpython.readthedocs.io/projects/servokit/en/latest/)

Adafruit Servokit is a Python library that provides a high-level interface with low-level PWM controls. For this package, the library is used to control PWM servos and a ESC connected to channels of the PCA9685 I2C breakout board.
more details <a href="https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython" >here</a>


### [cv_bridge](http://wiki.ros.org/cv_bridge)

CV Bridge provides functions to easily convert (encode/decode) in between ROS image message types to OpenCV-workable Numpy arrays.

## Environment Configuration

`sudo apt-get update`

`sudo apt-get upgrade`



**1. openCV Setup**

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

**IF NEEDED**

  **d. build instructions** <a href="https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html" >here</a>



**2. Virtual Environment Setup**

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

   **NOTE**
   _**WHILE IN VIRTUAL ENVIRONMENT, DO NOT USE "sudo" 
   TO INSTALL PIP PACKAGES, THESE WILL INSTALL TO YOUR 
   LOCAL MACHINE INSTEAD OF VIRTUAL ENVIRONMENT!!!**_

   more details <a href="https://realpython.com/python-virtual-environments-a-primer/" >here</a>
   and <a href="https://bic-berkeley.github.io/psych-214-fall-2016/using_pythonpath.html" >here</a>

**3. Upgrading**

`pip install pyyaml`

`pip install rospkg`

`pip install --upgrade pip`

`pip install --upgrade pyinstaller`



**4. Install Adafruit Library**

`pip install adafruit-circuitpython-pca9685`

`pip install adafruit-circuitpython-servokit`

more details <a href="https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython" >here</a>

**5. Install ROS Melodic**
Instructions found <a href="https://docs.google.com/document/d/1LxcTvSTRHVxSnv3x_cJ667loWgzCt7ikTJk51dKAFSs/edit?usp=sharing" >here</a>


**6. Access this repository**

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

## **Work Flow To Use This Repository**

1. **ALWAYS RUN ROSCORE IN A TERMINAL ON EVERY BOOT UP OF THE JETSON**

`roscore`

2. Calibrate camera using the [**Find Camera Parameters**](#find-camera-parameters) script. These values are saved automatically (see [**Find Camera Parameters**](#find-camera-parameters) for details) to a configuration file, so just press control-c when the camera is calibrated. Below are the commands to get the camera running and begin the calibration process.


`rosrun ucsd_robo_car_simple_ros camera_server.py`

`rosrun ucsd_robo_car_simple_ros camera_values_ros.py`


3. Calibrate throttle and steering using instructions in [Throttle and Steering Calibration](#throttle-and-steering-calibration). Find a throttle value for both the optimal condtion (error = 0) and the non optimal condtion (error !=0) AKA go fast when error=0 and go slow if error !=0 (see [**lane_guidance_node**](#lane_guidance_node) for details). Once both of these values are found, enter the values manually in **lane_guidance.py**
For steering, change the Kp value to adjust the steering sensitivty (as Kp --> 1 steering more responsive, as Kp --> 0  steering less responsive) Below are the commands for calibrating throttle and steering as well as editing **lane_guidance.py**

`rostopic pub -r 15 /steering [TAB][TAB]`

`rostopic pub -r 15 /throttle [TAB][TAB]`

`roscd ucsd_robo_car_simple_ros/scripts`

`nano lane_guidance.py`

enter the upper and lower bounds for throttle on lines 26 and 29 and the Kp for steering on line 17

Then press 
   
   **ctrl-x** 
   
   Then press 
   
   **y**  (yes) 
   
   and then press 
   
   **enter** 
   
   to save an quit


4. Experiment which algorithim is better; [**line_detection_node**](#line_detection_node) or [**lane_detection_node**](#lane_detection_node)

## Nodes

#### **throttle_client**

Associated file: throttle_client.py

This node subscribes to the [**throttle**](#Topics) topic. We use subscriber callback function
to validate and normalize throttle value, and then use the [**adafruit_servokit**](#adafruit_servokit)
module on **channel 0** for sending signals to the hardware.

This node is also responsible for reading and setting the throttle calibration values.

See [**throttle and steering calibration**](#throttle_and_steering_calibration) for calibration

#### **steering_client**

Associated file: steering_client.py

Similar to [**throttle_client**](#throttle_client), this node subscribes to the [**steering**](#Topics)
topic and passes the signals to the hardware. The steering servo is on **channel 1**.


Plenty of information on how to use the adafruit_servokit libraries can be found <a href="https://learn.adafruit.com/16-channel-pwm-servo-driver/python-circuitpython" >here</a> and <a href="https://github.com/adafruit/Adafruit_CircuitPython_ServoKit" >here</a> 

See [**throttle and steering calibration**](#throttle_and_steering_calibration) for calibration

#### **camera_server**

Associated file: camera_server.py

This node simply reads from the camera with cv2's interface and publishes the image to the
[**camera_rgb**](#Topics) topic. Before publishing, the image is reformatted from the cv image format
so it can be passed through the ROS topic message structure.


#### **line_detection_node**

Associated file: line_detection.py

This node subscribes from [**camera_rgb**](#Topics) topic and uses opencv to identify line
information from the image, and publish the information of the middle point of 
a single line to the [**centroid**](#Topics) topic. The color of line is chosen by the user
and set by using [**find_camera_values**](#tools)

Throttle is based on whether or not a centroid exists - car goes faster when centroid is present and slows down when there is none.

Steering is based on a proportional controller implemented by its error function. Gain (Kp) can be tuned in this script.


**Note: The cv windows have been commented out so that no errors occur when running in headless mode. For debugging, its suggested to uncomment these lines.**

#### **lane_detection_node**

Associated file: lane_detection.py

This node has the same functionality as [**line_detection_node**](#line_detection_node) however, now the ability to identify more than one line has been included. It is possible to identify the outside lanes as well as the yellow dashed lines if a green mask is applied which can easily be made by using [**find_camera_values**](#find_camera_values). 

**Note 1: The bounding areas found in the image can be calibrated visually using** [**Find Camera Parameters**](#find-camera-parameters)

**Note 2: The cv windows have been commented out so that no errors occur when running in headless mode. For debugging, its suggested to uncomment these lines.**

Below show the image post processing techniques, cv2 methods and the logic applied respectively.

<div>
  <img src="filtering_process.png">
  <img src="applying_methods.png">
  <img src="applying_logic.png">
</div>

For [**lane_detection_node**](#lane_detection_node), the logic above shows that the bounding boxes and centroids change color based on the number of contours found in the image. 

ie. 

- 2 contours : green bounding boxes each with their own green centroid and a blue average centroid
- 1 contour : green bounding box with a single red centroid

#### **lane_guidance_node**

Associated file: lane_guidance.py

This node subscribes to the centroid topic, calculates the throttle and steering
based on the centroid value, and then publish them to their corresponding topics.
Throttle is based on whether or not a centroid exists - car goes faster when centroid is present and slows down when there is none.
Steering is based on a proportional controller implemented by the calculating the error between the centroid found in [**line_detection_node**](#line_detection_node) or [**lane_detection_node**](#lane_detection_node) and the heading of the car. 

Gains can be tweaked in the lane_guidance.py script.

#### **camera_values_node**

Associated file: camera_values_ros.py

See [**Find Camera Parameters**](#find-camera-parameters) for details about this node.

## Topics


#### **throttle** 
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| throttle   | std_msgs.msg.Float32  | Float value from -1 to 1 for controlling throttle          |


#### **steering**
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| steering   | std_msgs.msg.Float32  | Float value from -1 to 1 for controlling steering          |

#### **camera_rgb**
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| camera_rgb | sensor_msgs.msg.Image | Image last read from USB camera image                      |

#### **centroid**
| Name       | Msg Type              | Info                                                       |
| ---------- | --------------------- | ---------------------------------------------------------- |
| centroid   | std_msgs.msg.Int32MultiArray    | arg 1: Integer for x coordinate of centroid in camera image space arg 2: camera width ex. [centroid,camera_width]|



## Launch

#### **throttle and steering launch**
This file launches both [**throttle_client**](#throttle_client) and [**steering**](#Topics) seperately because these topics can take some time to initialize which can delay productivity. Launch this script once and use the other launch files listed below to get the robot moving.

`roslaunch ucsd_robo_car_simple_ros throttle_and_steering_launch.launch`

#### **line Detection launch**

This file will launch [**line_detection_node**](#line_detection_node), [**lane_guidance_node**](#lane_guidance_node), [**camera_server**](#camera_server) and load the color filter parameters created using [**Find Camera Parameters**](#find-camera-parameters)

**Before launching, please calibrate the robot first while on the stand! See** [**Throttle and Steering Calibration**](#throttle-and-steering-calibration)

`roslaunch ucsd_robo_car_simple_ros lineDetection_launch.launch`

#### **lane Detection launch**

This file will launch [**lane_detection_node**](#lane_detection_node), [**lane_guidance_node**](#lane_guidance_node), [**camera_server**](#camera_server) and load the color filter parameters created using [**Find Camera Parameters**](#find-camera-parameters)

**Before launching, please calibrate the robot first while on the stand! See** [**Throttle and Steering Calibration**](#throttle-and-steering-calibration)

`roslaunch ucsd_robo_car_simple_ros laneDetection_launch.launch`

#### **ucsd_robo_car calibration launch**

This file will launch [**camera_server**](#camera_server) and [**Find Camera Parameters**](#find-camera-parameters)

`roslaunch ucsd_robo_car_calibration_launch.launch`

## Tools 

#### **Run Indvidual Programs**

To run any indvidual program, enter this into the terminal and change file_name.py to whatever python file is in the repo

`rosrun ucsd_robo_car_simple_ros file_name.py`

#### **Throttle and Steering Calibration**

To calibrate steering and throttle, using the commands below to test different values for throttle and steering angle. To make sure the right message is passed to topics, pressing the "TAB" key on the keyboard twice will autocomplete how the message should be structured and only the value at the end needs to be changed. 

**NOTE: Throttle is EXTREMELY sensitive. Start with very small values such as 0.01, 0.02, 0.03**

First launch the throttle and steering clients 

`roslaunch ucsd_robo_car_simple_ros throttle_and_steering_launch.launch`

Then in 2 new terminal windows enter these commands 

`rostopic pub -r 15 /steering [TAB][TAB]`

`rostopic pub -r 15 /throttle [TAB][TAB]`

Once throttle values are found, enter them in the [**lane_guidance_node**](#lane_guidance_node) as the values the car will go when it finds or doesnt find a line or lane to follow


#### **Find Camera Parameters** 

Associated file: camera_values_ros.py

This program allows for the user to quickly tune various camera post-processing parameters including a custom color filter. 
These values will **automatically** be sent to either the [**line_detection_node**](#line_detection_node) or the [**lane_detection_node**](#lane_detection_node) (depending on which you are using) by using rosparam functionality. These values are also written to a file called **custom_filter.yaml** which permanently stores these valus to be used at a later time so that this program does not have to be run again. 

To run this script:

`roslaunch ucsd_robo_car_simple_ros ucsd_robo_car_calibration_launch.launch`

OR

`rosrun ucsd_robo_car_simple_ros camera_server.py`

`rosrun ucsd_robo_car_simple_ros camera_values_ros.py`

Answer the promt _("Create green filter? (y/n) ")_ then hit enter and begin creating your custom filter and color tracker!



| Property       | Info                                                       |
| ---------- | --------------------- |
| Hue_low, Hue_high | Setting low and high values for Hue  | 
| Saturation_low, Saturation_high | Setting low and high values for Saturation | 
| Value_low, Value_high | Setting low and high values for Value | 
| Width_min, Width_max | Specify the width range of the line to be detected  | 

More morphological transfromations and examples can be found <a href="https://docs.opencv.org/3.4/db/df6/tutorial_erosion_dilatation.html" >here</a> and  <a href="https://docs.opencv.org/master/d9/d61/tutorial_py_morphological_ops.html" >here</a>

<!-- blank line -->
<figure class="video_container">
  <video controls="true" allowfullscreen="true" poster="path/to/poster_image.png">
    <source src="yellow_detect_v1_6_23.mp4" type="video/mp4">
  </video>
</figure>
<!-- blank line -->


#### **Decoder** 

Associated file: decoder.py

This provides a solution for cv_bridge not working and decodes the incoming image into a numpy array that is then passed to the [**camera_rgb**](#Topics) topic. If cv_bridge is built with python3, then this file is not neccessary.


## Issues and Fixes

### **Error With CV_Bridge Conversion From Image Message To OpenCV Image**

Using **bridge_object.imgmsg_to_cv2()** threw errors on our Jetson Nano environment, so we had to resort to our own image decoder function. Function **decodeImage()** can be imported from **decoder.py**. If you don't want to use our function, the problem can be avoided by properly building CV_Bridge with Python3 in the ROS package.

An alternative solution can be found <a href="https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674" >here</a>

### **Throttle Not working**

This issue can vary between cars, but generally the problem lies in the battery supply and the PWM range that is mapped by the Adafruit library. If the "start" PWM is too low, then even a maxed out "1" might not map to the PWM value that will trigger the ESC. First make sure the -1 to 1 range is properly calibrated. During runtime, the scale constant found in **throttle_client.py** can also be tuned. As your battery begins to drain, the PWM range becomes under-saturated which decreases performance of the motor. 

**Tip: Always try driving with fully charged battery or periodically recalibrate pwm values manually as motor performance starts decreasing.**


### **ROS Version Is Not Compatible with Python3**
If your're having issues using python3, then there is a chance that the virtual environment (explained in [**Environment Configuration**](#environment-configuration)) was not setup properly. Try setting up another environment to see if that solves the issue.

More info found 
<a href="https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674" >here</a>

