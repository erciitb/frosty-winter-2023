# Episode 2 - The Sign of ArUcos


So till now you have learnt how to control our bot and guide it through the maze using your keyboard. But when we are thinking about challenging Moriarty, we can't take any chances! We have to ensure that the bot is able to navigate it's own way through the maze using some clues present at each nodes of the maze. Holmes has deduced the nature of the clues that would be present. According to him, they would be of the form of markers known as ArUco markers. We need to train our bot to identify and process these. But before that, what exactly are this 'ArUco markers'? Let's find out...


## ArUco Markers
ArUco marker is a grid of black and white squares, typically a 5x5 grid, which looks something like this:

![This is an image](W2_Images/ArUco%20marker.png)

ArUco markers are used to for camera pose estimation, or calibration of the bot. In an ArUco marker, black box represents the number 0 and white box represents the number 1. So going by this, let us breakdown the above marker into grid. Also note that ArUco markers have a black border(padding) of 1 unit around them to make their detection easier, so that is neglected below.

![This is an image](W2_Images/Grid%20for%20aruco%20marker.png)

ArUco markers use parity to figure out the orientation of the marker. You may read more about parity bits [here](https://en.wikipedia.org/wiki/Parity_bit) and how it is used in ArUco markers [here](https://pdfcoffee.com/aruco-tutorial-pdf-free.html). [This](https://ria.ua.pt/bitstream/10773/23890/1/artigo.pdf) is how it is able to detect its orientation with the help of parity.

It's fine if you don't understand what parity bits are, as that part is taken care by the computer.


Alright, so now we understand what ArUco markers are, we need to find a way so that they are read by the bot through a camera fitted on it. This can be achieved through OpenCV.

## OpenCV

<img src="W2_Images/opencv_logo.png" width="320" height = "150">

OpenCV (Open-Source Computer Vision Library) is an open-source library that includes several hundreds of computer vision algorithms. It helps us in performing various operations on images very easily.

In our task we will be mainly using ArUco, an OpenCV based library for detecting the markers and navigating through the maze. Nevertheless, some basic knowledge of OpenCV might be very useful for some of your future projects in Gazebo and Rviz.

### Installation and Setup

**Note:** Ubuntu has two python versions Python 2 and Python 3, you can see exact version of them by running ```python``` and ```python3``` in Terminal respectively. ROS (upto Melodic) officially supports only python 2 and NOT python 3. ROS Noetic target Python 3 exclusively. 

Most of you are using ROS Noetic, so we will focus on that only. ROS Melodic users will have to change 'python3' to 'python' in the ```#!/usr/bin/env python3``` in the first line of .py files they are using. 

#### Installing OpenCV </br>
  
 Execute 
 ```bash
 pip --version
 ```
 Ensure that pip is configured with python3.xx . If not you may have to use (```pip2 --version``` or ```pip3 --version```).
If it shwos 'ImportError: No module named pip' or similar error, you need to install pip by executing
 ```bash
 pip install pip
 ```
Execute either
```bash
pip install opencv-contrib-python
#or
pip install opencv-python
```
Use ```pip2``` or ```pip3``` in the above commands, if python3 is configured with one of them.

Type ```python3``` in Terminal to start Python interactive session and type following codes there.
```bash
import cv2 as cv
print(cv.__version__)
```
If the results are printed out without any errors, congratulations !!! You have installed OpenCV-Python successfully.

You may Install OpenCV from source. (Lengthy process)

Please refer to this [link](https://docs.opencv.org/4.5.0/d2/de6/tutorial_py_setup_in_ubuntu.html). This installation can take some time so have patience.

#### Setting up VS code
  
```bash
cd ~/catkin_ws/src
git clone https://github.com/kaushaljadhav512/opencv_tutorials
```
Launch VSC and in the explorer’s tab open the newly created folder (opencv_tutorials) and create a .py file to write your practice code.
Open the Extensions tab on the left side of your screen. Search and install the python extension.

Congrats! Now we are all set. For this workshop we will be using standard functionality of OpenCV for ArUco markers. To learn basic commands and uses of OpenCV and how it is used in image processing, go [here](https://github.com/erciitb/Image-processing).

## Libraries for ArUco Markers 

Let us see how can we use OpenCV to detect ArUcos. By detecting, here our objective is to detect the position of the corners of the marker and ID of the marker(this is different from the data bits number, read on to find out). We will use the python library - ArUcos. In the header of your python script, add the following libraries:

```python
import numpy as np
import math
import cv2
import cv2.aruco as aruco
```
`aruco` library has predefined dictionaries of markers, which it uses to detect the given markers. We have to create an instance of these dictionaries before we proceed. It is done using:

```python
aruco_dict = aruco.Dictionary_get(aruco.DICT_5x5_250)
```
_If due to some reason, you are getting an error in the above line of code, you can replace it by:_
```python
aruco_dict = aruco.Dictionary_get(getattr(aruco, f'DICT_{5}X{5}_{250}'))
```
Moving on, this is an example of a dictionary of 250 ArUco markers of size 5x5. 

Let us say the image we have got from the camera is stored in the variable `img`. (We will discuss how to get the image from camera in ROS later) \
Also remember that it is okay to have more than one ArUco markers in an image.

Ok, so now lets convert this image into grayscale image and store it into another variable `gray`.
```python
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
```
Next we create an instance of a class, which includes all the options that can be customized during the marker detection process:
```python
Parameters = aruco.DetectorParameters_create()
```
And finally, we unpack parameters of the marker through:

```python
corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = Parameters)
```
Note that we have used " _ " above because there is a third parameter which is returned above and we aren't interested in it.

So let us see what are the arguments:
- gray : Grayscale image of the sample to be detected.
- aruco_dict : The dictionary of which the ArUco marker is a part of.
- Parameters : This is the object returned by the aruco.DetectorParameters_create()

What this returns is:
- ids : This is a list, which contains the ArUco id according to the dictionary; if there are N markers in the image, then size of the list is N.
- corners : This is a numpy array of the 2D coordinates of the corners of the markers. For each marker, the four corners are returned in their _**original**_ order, i.e. clockwise starting from top right(This info will come handy later, remember it). If there are N markers in the image, then the size of the array(i.e. _corners_) is Nx4.

## cv_bridge

Now that we are familiar with the basics of OpenCV, ArUco and ROS, we can finally talk about integrating these two and performing various operations on images such as image detection.</br>

Unfortunately, in ROS, the format of the images being processed (ROS image Message) is quite different than that used in OpenCV (cv::Mat). This is where the library  cv_bridge comes to the rescue! </br>
We create a publisher-subscriber model to import and export images out of ROS into OpenCV and back into ROS. 

<img src="W2_Images/cvbridge3.png" width="300" height="330">

### Let us start with a simple example.
Suppose, we are getting Image data on ```/camera/rgb/image_raw``` topic. 
Here is a node that listens to a ROS image message topic, converts the images into an cv::Mat, displays the image using OpenCV. 

```python
#!/usr/bin/env python3  #setting up environment
  
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


def callback(img_msg):
    # Initialize the CvBridge class
    bridge = CvBridge()
    # Print some info of image to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")   # this function converts images format used
								  # in ROS to the one used in CV
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Convert the image to Grayscale
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)  # cv2.cvtColor(img_in , color code) converts the image 
    						       # according to the color code. For example, 
    						       # RGB to GBR to Grayscale etc. Refer this for more info. and codes
						       # https://docs.opencv.org/4.x/d8/d01/group__imgproc__color__conversions.html
    # Show the converted image
    cv2.namedWindow("Image Window", 1)    # creating a window in which the image will be displayed
    cv2.imshow("Image Window", gray)      # displaying the new image in that window
    cv2.waitKey(3)			  # wait 3 seconds for a user input


def laser():
    rospy.Subscriber('/camera/rgb/image_raw', Image, callback)  
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cvbridge_example', anonymous=True)
    try:
        laser()

    except rospy.ROSInterruptException:
        pass
```


## Proceeding to detect ArUco..

Open the Terminal and run following commands-
```bash
cd ~/catkin_ws/src
git clone -b python3_noetic --single-branch https://github.com/Tejas2910/aruco_detection
cd ~/catkin_ws
catkin_make
```
Now you have a package aruco_detection, let's run it.
```bash
roslaunch aruco_detection maze_aruco.launch
```
Let's spawn the Turtlebot3 by running following command in another tab
```bash
roslaunch aruco_detection spawn_turtlebot3.launch
```
You can see ArUco marker in front of TurtleBot3(waffle_pi model).
Why we used waffle_pi ? Guess... Remember Investigation 1 of Episode 1. 

Yes, you guessed correctly. Let's check by executing ``` rostopic list ``` in another tab.
```bash
/clock
/cmd_vel
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/set_link_state
/gazebo/set_model_state
/imu
/joint_states
/odom
/rosout
/rosout_agg
/scan
/tf
/turtlebot3_waffle_pi/camera/camera_info
/turtlebot3_waffle_pi/camera/image_raw
/turtlebot3_waffle_pi/camera/image_raw/compressed
/turtlebot3_waffle_pi/camera/image_raw/compressed/parameter_descriptions
/turtlebot3_waffle_pi/camera/image_raw/compressed/parameter_updates
/turtlebot3_waffle_pi/camera/image_raw/compressedDepth
/turtlebot3_waffle_pi/camera/image_raw/compressedDepth/parameter_descriptions
/turtlebot3_waffle_pi/camera/image_raw/compressedDepth/parameter_updates
/turtlebot3_waffle_pi/camera/image_raw/theora
/turtlebot3_waffle_pi/camera/image_raw/theora/parameter_descriptions
/turtlebot3_waffle_pi/camera/image_raw/theora/parameter_updates
/turtlebot3_waffle_pi/camera/parameter_descriptions
/turtlebot3_waffle_pi/camera/parameter_updates
```
Camera Sensor is publishing data of ```sensor_msgs/Image``` msg type to ```/turtlebot3_waffle_pi/camera/image_raw``` topic. Let's visualize this data throgh **Rviz**.

Run ```rviz``` in Terminal. Click on Add button, Under tab **By topic** add ```/turtlebot3_waffle_pi/camera/image_raw``` topic. You can see data published on this topic.  

<img src="W2_Images/Rviz_CameraTopic.jpg" width=250 height=400>

Now, we will subscribe ```/turtlebot3_waffle_pi/camera/image_raw``` topic to convert ROS Image data to OpenCV Image data using **cv_bridge**.

Execute the following command in another tab.
```bash
rosrun aruco_detection detect_marker.py
```
On executing You should be able to see following screen.

<img src="W2_Images/cv_bridge_1.jpg" width=600 height=300>

Have a look at the detect_marker.py file

```python
#!/usr/bin/env python3

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return:   dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners},
	## 	     where ArUco_id_no indicates ArUco id and corners indicates the four corner position 
	##	     of the aruco(numpy array)
	##	     for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163], [319, 263], [219, 267], [215,167]], dtype=float32)}
						
    Detected_ArUco_markers = {}
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)    		# converting image to grayscale
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)	# extracting a pre-defined dictionary of various aruco markers
    parameters = aruco.DetectorParameters_create()		# returns the parameters required by opencv to detect aruco markers. Leave it at default
    
    # get the coordinates of all the aruco markers in the scene along with their ID
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters) 
    i = 0
    
    # storing extracted data
    try:
        for id in ids:
            for id_Number in id:
                Detected_ArUco_markers[id_Number] = corners[i][0]    

    except TypeError:
        print("No aruco in front of me")

    i += 1
    return Detected_ArUco_markers


def mark_ArUco(img,Detected_ArUco_markers):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	## return: image helping sherlock to solve maze 

    ids = Detected_ArUco_markers.keys()   # IDs of all aruco markers in the scene
    print(Detected_ArUco_markers)
    centre_aruco = {}
    top_centre = {}
	
    # drawing circles and line for each aruco marker by using coordinates found earlier
    try:
        for id in ids:
            corners = Detected_ArUco_markers[id]
            for i in range(0, 4):
                cv2.circle(img,(int(corners[i][0]), int(corners[i][1])), 5, (0,0,255), -1)
            centre_aruco[id] = (corners[0]+corners[1]+corners[2]+corners[3])/4
            top_centre[id] = (corners[0]+corners[1])/2
            cv2.line(img, (int(centre_aruco[id][0]), int(centre_aruco[id][1])),
	    		(int(top_centre[id][0]), int(top_centre[id][1])), (255, 0, 0), 5)

    except TypeError:
        print("No aruco in front of me")

    return img

def callback(img):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img, "bgr8")       # convering imahe to CV2 usable format
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))      
    Detected_ArUco_markers = detect_ArUco(cv_image)	   # Dictionary of detected markers
    img = mark_ArUco(cv_image,Detected_ArUco_markers)      # Image marked with circles and line for each aruco marker
    cv2.namedWindow("Image Window", 1)       		   # creating named window
    cv2.imshow("Image Window", img)			   # displaying edited image
    k = cv2.waitKey(1)					   # waiting 1 second for user input
    

def laser():
    rospy.Subscriber('/turtlebot3_waffle_pi/camera/image_raw', Image, callback)
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('detect_marker')
    try:
        laser()

    except rospy.ROSInterruptException:
        pass
	
```
Run ```roslaunch aruco_detection turtlebot3_teleop_key.launch``` in another window, and try to move the bot.

Now, we have seen ArUco detection,

# Let's Solve mAzE

At this stage, you have enough knowledge to escape from the maze created by Moriarty.

Open **maze_aruco.launch** file in launch folder and replace empty.world with maze_aruco.world. Required file is:

```xml
<launch>

  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find aruco_detection)/worlds/maze_aruco.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

</launch>
```
Execute following command 
```bash
roslaunch aruco_detection maze_aruco.launch
roslaunch aruco_detection spawn_turtlebot3.launch
```
Upon execution, the following screen should be visible.

<img src="W2_Images/maze.jpg" width=600 height=300>

Cool !

How will you come out of this maze, which is surrounded by walls from all the sides ?

Well, it's Moriarty's maze.

There is a trick - Bot can go through some of the walls present in the maze. But, how will bot find those walls ? 

ArUco says hi!! 

ArUco will guide you along the way to solve the maze. 

<img src="W2_Images/aruco_guide.jpg" width=600 height=300>

The **Blue line** in ArUco marker in "Image Window" is indicating that magic wall 

Execute ```rosrun aruco_detection detect_marker.py```. Open new terminal and execute ```roslaunch aruco_detection turtlebot3_teleop_key.launch``` to control bot.

If you are curious about how these walls are created, don't worry. We will go deeper into these things in coming week.

Now, go ahead and solve the maze. :)

### That's the end of Week 2! Cheers!

<img src="W2_Images/sherlock_toast.jpg" height="300" width="450">
