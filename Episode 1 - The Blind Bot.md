# Episode 1 - The Blind Bot

## Introduction

If you have gone through the content of Week 0 and tried the problem, you should be familiar with the basic ideas of ROS. In addition, you should be capable of creating a simple publisher and a subscriber. If so, you are ready to face what is about to come your way. In this episode, you will see how to work in **Gazebo** and **Rviz**. 


<img src="W1_Images/Gazebo.jpg" width=320 height=150> <img src="W1_Images/Rviz.png" width=320 height=150>

You will also get to play with the **Mrs_Hudson** in Gazebo and see the working of its sensors in Rviz2.

<img src="W1_Images/mrs_hudson.jpeg" width=200 height=300>

Let's begin !

## Table of Contents

<ol>
  <li><a href="#Gazebo">Gaze at Gazebo ...</a></li>
  <li><a href="#Rviz">Viziting Rviz ...</a></li>
  <li><a href="#mrs_hudson">mrs_hudson emerges ...</a></li>
</ol>

## Initial preparation

Create a package ```week1_tutorials``` in ```erc_ws```, with ```launch```, ```models```, ```worlds``` and ```configs``` folders. This will be required for storing the various files that will be created throughout. Recall how to make a package from week 0. 

## Gaze at Gazebo ... <a name="Gazebo"></a>


### What is it ?

Gazebo is a **robotics simulator** that is capable of simulating **dynamical systems** in various **realistic scenarios and environments**. 
This is useful for testing algorithms and designing robots before actually implementing in the real physical world.We will be using Gazebo Fortress.


### Installing Gazebo

First install some necessary tools:


```bash
sudo apt-get update
sudo apt-get install lsb-release wget gnupg
```
Then install Ignition Fortress:

```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install ignition-fortress
```

Launch Gazebo Sim, a 3D robotics simulator, from a terminal by running:

```bash
ign gazebo sim shapes.sdf
```

<img src="W1_Images/gazebo_interface.png" width=600 height=400>


Welcome to Gazebo !

### Basic features

Refer to the following link to know about the basic GUI features of Gazebo.

[Understanding the GUI](https://gazebosim.org/docs/harmonic/gui)

### Manipulating Models : The Transform Control

The top left toolbar contains controls for transforming. 

1. Open Gazebo
2. The View Angle plugin allows you to quickly and easily adjust the direction from which your scene faces an entity in the simulation.
3. The Component Inspector plugin displays a variety of object attributes.

4.  The Align Tool aligns entities along or about a specific entity's bounding box.

5.  Select Mode: Click to select entities in the scene. It is the default mode. To select multiple entities, hold ```Ctrl``` and click. Entities can't be manipulated in select mode. 
    You can always return to selection mode from any other mode by pressing Esc.

    <img src="W1_Images/select.png" width=500 height=500>

6.  Translate  Mode: Allows to translate entities along the x, y and z axes.(Shortcut ```T```)

    <img src="W1_Images/translate_icon.png" width=500 height=100>


    <img src="W1_Images/translate.gif" width=500 height=500>


7.  Rotate  Mode: Allows to rotate entities around the roll, pitch and yaw axes of rotation.(Shortcut ```R```)

    <img src="W1_Images/rotate_icon.png" width=500 height=100>

    <img src="W1_Images/rotate.gif" width=500 height=500>



### Keyboard Shortcuts


<img src="W1_Images/key_shortcuts.png" width=700 height=350>


#### Building your own Robot

##### SDF
SDFormat (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, visualization, and control.

We will start by building a simple world and then build our robot in it. Open a new file called cub_world.sdf and copy the following code to it.

```bash
<?xml version="1.0" ?>
<sdf version="1.8">
    <world name="cub_world">
        <physics name="1ms" type="ignored">
            <max_step_size>0.001</max_step_size>
            <real_time_factor>1.0</real_time_factor>
        </physics>
        <plugin
            filename="libignition-gazebo-physics-system.so"
            name="ignition::gazebo::systems::Physics">
        </plugin>
        <plugin
            filename="libignition-gazebo-user-commands-system.so"
            name="ignition::gazebo::systems::UserCommands">
        </plugin>
        <plugin
            filename="libignition-gazebo-scene-broadcaster-system.so"
            name="ignition::gazebo::systems::SceneBroadcaster">
        </plugin>

        <light type="directional" name="sun">
            <cast_shadows>true</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
            <attenuation>
                <range>1000</range>
                <constant>0.9</constant>
                <linear>0.01</linear>
                <quadratic>0.001</quadratic>
            </attenuation>
            <direction>-0.5 0.1 -0.9</direction>
        </light>

        <model name="ground_plane">
            <static>true</static>
            <link name="link">
                <collision name="collision">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    </plane>
                </geometry>
                </collision>
                <visual name="visual">
                <geometry>
                    <plane>
                    <normal>0 0 1</normal>
                    <size>100 100</size>
                    </plane>
                </geometry>
                <material>
                    <ambient>0.8 0.8 0.8 1</ambient>
                    <diffuse>0.8 0.8 0.8 1</diffuse>
                    <specular>0.8 0.8 0.8 1</specular>
                </material>
                </visual>
            </link>
        </model>
    </world>
</sdf>
```


Save the file in the ```models``` directory of the package, navigate to the directory and launch the simulator:

```ign gazebo cub_world.sdf```


#### Building a cuboidal box

Under the </model> tag we will add our robot model as follows:

```bash
<model name='cuboidal_model' canonical_link='cuboid'>
    <pose relative_to='world'>0 0 0 0 0 0</pose>
```

Every model is a group of links (can be just one link) connected together with joints.

```bash
 <link name='cuboid'>
        <pose relative_to='__model__'>0.5 0 0.4 0 0 0</pose>
```
Inertial properties

```bash
  <inertial> <!--inertial properties of the link mass, inertia matix-->
        <mass>1.14395</mass>
        <inertia>
            <ixx>0.095329</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.381317</iyy>
            <iyz>0</iyz>
            <izz>0.476646</izz>
        </inertia>
    </inertial>
```
Visual

```bash
    <visual name='visual'>
        <geometry>
            <box>
                <size>2.0 1.0 0.5</size>
            </box>
        </geometry>
        <!--let's add color to our link-->
        <material>
            <ambient>0.0 0.0 1.0 1</ambient>
            <diffuse>0.0 0.0 1.0 1</diffuse>
            <specular>0.0 0.0 1.0 1</specular>
        </material>
    </visual>
```

Collision

```bash
        <collision name='collision'>
            <geometry>
                <box>
                    <size>2.0 1.0 0.5</size>
                </box>
            </geometry>
        </collision>
    </link>
</model>
```

After copying all the parts above into the world file in order, run the world again:

```ign gazebo cub_world.sdf```


#### Spawning models using launch file

If you want to spawn the models when launching gazebo world. Make a launch file ```cub.launch.py``` in the launch folder in the package and add the following code

```py
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	
	pkg_project = get_package_share_directory('week1_tutorials')
	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

	gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': os.path.join(pkg_project, 'models', 'cub_world.sdf')}.items(),
    	)
	
	bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/world/empty/model/cub_world/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',
    				'/clock@rosgraph_msgs/Clock@ignition.msgs.Clock'],
        output='screen'
    )

	return LaunchDescription([
		gz_sim,
		bridge
		])
```

Now add the following line in ```setup.py``` in the ```data_files```

```python
(os.path.join('share', package_name, 'models'), glob(os.path.join('models', 'cub_world.sdf'))),
(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'cub.launch.py'))),
```

and add the follwing on the top of ```setup.py```

```python 
import os
from glob import glob
```

Now execute ```ros2 launch week1_tutorials cub.launch.py``` after ```colcon build``` form workspace to launch world and spawn the model into it.
  
### Plugins
  
Plugins are a dynamically loaded chunk of code. For example:

```bash
<plugin
    filename="libignition-gazebo-physics-system.so"
    name="ignition::gazebo::systems::Physics">
</plugin>
```

The ```Physics``` plugin is very important for simulating the dynamics of the world.

```bash
<plugin
    filename="libignition-gazebo-user-commands-system.so"
    name="ignition::gazebo::systems::UserCommands">
</plugin>
```

The ```UserCommands``` plugin is responsible for creating models, moving models, deleting them and many other user commands.

```bash
<plugin
    filename="libignition-gazebo-scene-broadcaster-system.so"
    name="ignition::gazebo::systems::SceneBroadcaster">
</plugin>
```

```SceneBroadcaster``` shows our world scene.

## Viziting Rviz2 ... <a name="Rviz"></a>

### What is it ?

Rviz2 is a **3D visualizer** for **ROS** that lets us view a lot about the **sensing**, **processing** and **state** of a robot.
This makes the **development** of robots easier and also enables us to **debug** more efficiently (better than looking at numbers on a terminal :P)

### What is the difference between Rviz2 and Gazebo ?

Rviz2 is a **visualizer** i.e it shows what the robot **perceives** is happening while Gazebo is a **simulator** i.e. it shows what is **actually** happening.

Consider the scenario in which we do not have physical hardware-based robots. In that case we would use a simulator like Gazebo to know what would actually happen and the data from the sensors can be visualized in a visualization tool like Rviz2. In case we have physical robots, then the data from the sensors can still be visualized in Rviz, but we do not need a simulator necessarily.

### Installation

Execute the following command

```
sudo apt-get install ros-humble-rviz2
```

### Starting Rviz

```
rviz2
```
Upon execution, the following screen should appear.

<img src="W1_Images/rvizscreen.png" width=600 height=400>

Welcome to Rviz2 !

### Basic features


1. **Displays** - These are entities that can be "displayed"/ represented/visualized in the world like **point cloud** and **robot state**

<img src="W1_Images/displays.png" width=200 height=400>

Using the **Add** button, we can add additional displays.

<img src="W1_Images/adddisplays.png" width=200 height=400>


2. **Camera types** - These are ways of viewing the world from different angles and projections.

<img src="W1_Images/c
ameratypes.png" width=300 height=90>

3. **Configurations** - These are combinations of displays, camera types, etc that define the overall layout of what and how the visualization is taking place in the rviz window.

### Saving and Loading configurations

Currently the default layout of the Rviz window is similar to the picture below

<img src="W1_Images/deflayout.png" width=800 height=600>

Say we are interested in saving a configuration consisting of additional displays such as LaserScan as well as a different camera type. How do we accomplish that ?

1. Add the required displays and change the camera type

<img src="W1_Images/newlayout.png" width=800 height=600>

2. To save the configuration,

    2.1) Go to **File > Save Config As (Ctrl + Shift + S)**
  
    2.2) Go to the appropriate folder (```week1_tutorials > configs```), give an appropriate name (```custom```) and save

3. To load the configuration at a later time,

    3.1) Go to **File > Open Config (Ctrl + O)**
  
    3.2) Go to the appropriate folder (```week1_tutorials > configs```) and select the config file (```custom```)
   

### Starting Rviz2 through Launch files

Create ```custom_rviz.launch.py``` in the ```launch``` folder

Add the following code to launch Rviz with ```custom.rviz``` configuration

```python
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os.path

def generate_launch_description():
    pkg_project = get_package_share_directory('week1_tutorials')
    return LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_project, 'config', 'custom.rviz')]
        )
    ])
```

Now add the following line in ```setup.py``` in the ```data_files```

```python
(os.path.join('share', package_name, 'config'), glob(os.path.join('config', 'custom.rviz'))),
(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'custom_rviz.launch.py'))),
```

On executing  ```ros2 launch week1_tutorials custom_rviz.launch``` after `colcon build` from workspace, Rviz will be launched with the desired configuration.

## ros_gz 

ros_gz is a collection of packages that integrate ROS and Gazebo, enabling seamless communication between the two platforms. It provides features like bidirectional data exchange, image transport, point cloud publishing, and convenient tools for easy setup and use. By simulating robots in a virtual environment, ros_gz accelerates development, enhances debugging, and expands the possibilities of ROS-based robotics.

To install add https://packages.ros.org

```bash
 sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
 curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
 sudo apt-get update
```

Install ```ros_gz```

```bash
 sudo apt install ros-humble-ros-gz
```

## mrs_hudson emerges ... <a name="mrs_hudson"></a>

Meet mrs_hudson, our Sherlock-themed bot for hands-on ROS, Gazebo, and Rviz exploration. You'll find meticulously crafted URDF and launch files, enabling seamless integration for accomplishing investigations and unfolding mysteries


## Installing mrs_hudson

For cloning the ```mrs_hudson``` repo you need to have git installed on your system

```bash
apt-get install git
```

Then go to the ```erc_ws/src``` directory and open the terminal and run

```bash 
git clone https://github.com/erciitb/mrs_hudson.git
```

Now before building the package with ```colcon build``` first we need to change the path of meshes in the urdf and sdf files.

In the ```models``` and ```world``` folder open the urdf and sdf files and in mesh filename update the path of the meshes.

(PS: Use a simple find and replace, eg 

find = ```home/saurabh42/erc_ws/src/mrs_hudson/meshes```

replace = ```home/shiwani418/erc_ws/src/mrs_hudson/meshes```

Do a replace all in both the URDF and SDF file.
)

Then run ```colcon build``` in the workspace.

You can visit the [mrs_hudson repo](https://github.com/erciitb/mrs_hudson.git) on the ERC GitHub page.

Henceforth, the **mrs_hudson** may be referred to as **bot** simply, unless specified.

Let us see the bot in action in Gazebo !

## Launching mrs_hudson in Gazebo

To summon the bot in an **empty world** in **Gazebo**, execute the following command in the erc_ws directory.

```
ros2 launch mrs_hudson mrs_hudson_empty_world.launch.py
```

## Visualizing the bot

To visualize it in **Gazebo** & **Rviz**, run the following command in a separate tab

```
ros2 launch mrs_hudson mrs_hudson_gazebo_rviz.launch.py
```
<img src="W1_Images/newlayout.png" width=800 height=600>

## Taking a peek at the nrs_hudson topics

After launching the mrs_hudson in Gazebo, execute ```ros2 topic list``` in another tab.

The expected output is as follows
```
/clicked_point
/clock
/cmd_vel
/depth_camera
/depth_camera/points
/goal_pose
/initialpose
/joint_states
/lidar
/lidar/points
/model/mrs_hudson/odometry
/parameter_events
/robot_description
/rosout
/tf
/tf_static
/world/empty/model/mrs_hudson/joint_state
saurabh42@saurabh42-HP-Pavilion-Laptop-15-
```


<img src="W1_Images/Sherlock_moving.gif">

## Moving the bot around

Let's move the bot around in the standard world in Gazebo using the ```mrs_hudson_teleop``` script

The Turtlebot3 motion is described by its **linear velocity** and **angular velocity**. The ratio of the instantaneous linear velocity to the instantaneous angular velocity gives the **radius of curvature** of the arc it traverses at the instant.

First launch the bot in gazebo using the launch file then open a new terminal in the erc_ws directory and run the following command

```
ros2 run mrs_hudson mrs_hudson_teleop.py
```
we get the ability to control the linear velocity and the angular velocity of the bot using the appropriate keys as displayed on the screen.
```bash
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
```

One might quickly realize that moving the bot with the keys is kind of annoying.

Let's see another way of moving the bot around using a publisher that will publish velocity commands to the ```/cmd_vel``` topic. For simplicity, we shall make it go with a constant speed in a circular path to give the basic idea.

#### Knowing the message type

How do we know the type of message that needs to be published into ```/cmd_vel``` ? Well, on launching the bot in Gazebo, execute the following command in a new tab

``` ros2 topic info /cmd_vel ```

The expected output is ```geometry_msgs/Twist```

To inspect the nature of this message type, execute the following command

``` ros2 interface show geometry_msgs/msg/Twist ```

The expected output is

```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```


Once we know the features of the message we are dealing with, we can proceed with the code for moving the bot. 

```bash 
ros2 run mrs_hudson vel_pub.py
```
The bot begins to move in a line. Cool!


At this point, the bot must be feeling lonely roaming all by itself. Let us bring a friend to the world. Even a high-functioning sociopath needs one :D 

## Investigation

Take a look at the code in ```mrs_hudson_empty_world.launch.py```, ```mrs_hudson_gazebo_rviz.launch.py``` and ```mrs_hudson_teleop.py```. It will be helpful for the upcoming sections as the commands in these files will be used more or less directly with slight modification to launch the bots.

<img src="W1_Images/Sherlock_moving_2.gif">


mrs_hudson_empty_world.launch.py:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	
	pkg_project = get_package_share_directory('mrs_hudson')
	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

	gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': os.path.join(pkg_project, 'worlds', 'mrs_hudson.sdf')}.items(),
    	)
	
	bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/model/mrs_hudson/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/world/empty/model/mrs_hudson/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model',
                   'lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                   ],
        output='screen'
    )

	return LaunchDescription([
		gz_sim,
		bridge
		])
```

mrs_hudson_gazebo_rviz.launch.py`
```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
	
	pkg_project = get_package_share_directory('mrs_hudson')
	pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
	sdf_file = os.path.join(pkg_project, 'models', 'mrs_hudson.urdf')
	with open(sdf_file, 'r') as infp:
 		robot_desc = infp.read()
 	
	robot_state_publisher = Node(
  	 package='robot_state_publisher',
   	 executable='robot_state_publisher',
   	 name='robot_state_publisher',
   	 output='both',
   	 parameters=[
   	     {'use_sim_time': True},
   	     {'robot_description': robot_desc},
   	 ]
	)
	
	gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project, 'launch', 'mrs_hudson_empty_world.launch.py'))
    	)
	rviz = Node(
	package='rviz2',
	executable='rviz2',
	arguments=['-d', os.path.join(pkg_project, 'config', 'mrs_hudson.rviz')],
	)

	return LaunchDescription([
		robot_state_publisher,
		rviz,
		gz_sim
		])
```

mrs_hudson_teleop.py

```python
import rclpy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

settings = termios.tcgetattr(sys.stdin)

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0,0),
        'o':(1,0,0,-1),
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'u':(1,0,0,1),
        ',':(-1,0,0,0),
        '.':(-1,0,0,1),
        'm':(-1,0,0,-1),
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        't':(0,0,1,0),
        'b':(0,0,-1,0),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def main(args=None):    
    if args is None:
        args = sys.argv

    rclpy.init(args=args)
    node = rclpy.create_node('teleop_twist_keyboard')
        
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = 0.5
    turn = 1.0
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = th*turn
            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
```


## Lidar & mrs_hudson

Git clone the latest mrs_hudson package and then explore about lidar of the bot. 

Launch mrs_hudson using the ```mrs_hudson_gazebo_rviz.launch.py``` and observe the lidar data in rviz by moving the bot using the ```teleop``` script and adding some obstacles like cubes in the sdf file. :) just see how we spawn the cuboid in the empty world.

For observing the lidar in gazebo you can search lidar in gazebo and the click on it to observe the lidar. And for echoing lidar messages use:
```bash
ign topic -e -t /lidar
```

PS: See the ```mrs_hudson_arena.sdf``` provided for the assignment for better understanding of the lidar.

## Way ahead

Now that you have gained the ability to write code to move the bot around and sense the surroundings, what you can do with the bot is restricted only by your imagination. 

Try to know more about the mrs_hudson and explore various capabilities like navigation and SLAM by refering to blogs and documentations online.

Additionally, one can try writing code for publishers and subscribers in different ways apart from the prescribed style, such as using **classes**. We shall leave that up to you for exploration. Have fun.

# Let's play a game, shall we ... 

mrs_hudson is trapped in a room and there doesn't seem to be a way out unless the code to escape the room is figured out. She needs to **explore the room autonomously** and find clues which will help them determine the code. As they explore, they should make sure to **avoid colliding with objects** around them.

<img src="W1_Images/task1.png">
<img src="W1_Images/task1_lidar.png">

## Steps
1. Create a package ```task_1``` with ```launch```, ```config```, ```meshes``` and ```worlds```  folders.
2. Download the ```mrs_hudson_arena.world``` fand ```mrs_hudson_world.launch.py``` from the link below and add them to the ```worlds``` and ```launch``` folder respectively. Also think where to save the rviz configuration file.

    [Task 1_files](https://github.com/erciitb/frosty-winter-2023/tree/main/Task%201_files)

3. Create a node file ```96.py``` in the ```task1``` folder of ```task_1``` package, which will be responsible for **obstacle avoidance and exploration** of the room.
4. Then setup the package by adding the required in setup.py and package.xml. :) hope you guys understood and not just copied pasted till now. You need to add the meshes in task1 package, just copy formo mrs_hudson package. Also you need to change the directory of the meshes according to where you make the package in sdf file, as we did for setting up mrs_hudson package.
4. Launch he bot and your script and let the bot find its way.
5. The bot will begin exploring the room while avoiding obstacles.

Have fun!

PS: Hint use lidar data, write the python script to get the data and using it to find the path, it need not to be the most efficient for now.

<img src="W1_Images/Code.gif">

## Submission details

### Tentative deadline
19 December 2022, 11:59 PM

### Mode of Submission

The submission link is attached below.  Add ```96.py``` to a Google Drive folder and submit the link to the folder through the form. Also add the screen recording of the bot moving through the obstacle.

[Submission Link](https://forms.gle/xmkVyjG1QBQBTh3k7)


Also, make sure to change the settings as follows-

**Share > Get link > Anyone with the link**


### Points to be noted 

1. Submitting a simple algorithm that does the basic task of avoidance and exploration is good enough for this task. You will realize over time that a simple implementation might not be perfect in avoiding all kinds of obstacles since the obstacles can be in all shapes and orientations. You can experiment, test in different environments and improve the algorithm over time if the problem of obstacle avoidance and exploration continues to interest you.

2. If you are in a team, it is advised that you work with your team member for greater efficiency.