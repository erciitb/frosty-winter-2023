# Episode 0 - A Study in ROSe

## Introduction

**ROS**, which means the Robot Operating System, is a set of software libraries and tools to help you build robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more. The point of ROS is to create a **robotics standard**, so you don't need to reinvent the wheel anymore when building new robotic software.

ROS 2 builds on the concepts of ROS 1, re-architected using modern middleware for improved efficiency, reliability and flexibility on a wider range of platforms including embedded devices. Key concepts in ROS 2 include:

* Graph architecture - Nodes communicate via topics, services or actions to share data and commands
* Real-time capabilities - Optimized for complex robotics use cases and real-time code
* Expanding hardware support - Runs on embedded devices like Raspberry Pi, microcontrollers and FPGAs

This tutorial will cover the core ROS 2 concepts including working with nodes, topics, services, parameters and more through practical examples using the ROS 2 Humble release.

<img src="W0_Images/ros2.png " width=400 height=100>

## Table of Contents:

<ol>
	<li> <a href="#Preliminary"> Preliminary Installation </a> </li>
	<li> <a href="#Getting"> Getting started with ROS </a> </li>
	<li> <a href="#PubSub">Publisher-Subscriber Interface </a></li>
	<ol>
		<li><a href="#Publisher"> Writing a simple Publisher Node </a></li>
		<li><a href="#Subscriber"> Writing a simple Subscriber Node </a></li>
	</ol>
		
</ol>


## Preliminary Installation <a name="Preliminary"></a>
**WARNING** - The following installation procedures can make you do stuff like this ...

<img src="W0_Images/Sherlock_beating.gif" height=300> <img src="W0_Images/Sherlock_screaming.gif" height=300>

or even worse.

Refer to [FAQs](https://github.com/erciitb/frosty-winter-2023/blob/main/FAQs.md) for any issues or doubts regarding installation **first**.

* __Ubuntu Installation__ :
For using ROS2 framework, Ubuntu is not necessary, it can work on Mac and Windows also, but highly recommended. So, follow any of the five alternatives for setting up the linux environment:
(It's Preferable that you install Ubuntu 22.04)<br />

	**Dual-boot**: Follow this [Tutorial](https://www.tomshardware.com/how-to/dual-boot-linux-and-windows-11) or this [Video Tutorial](https://youtu.be/QKn5U2esuRk?si=RP5TieFTjEVU240-) to dual-boot Ubuntu with Windows. For MacOS, follow the procedure in this [video tutorial](https://youtu.be/jbUulXVZIBI?si=XTMyoI4yP6OC0Jc5) </li>
<span style="color:red">[WARNING], Do at your own risk! We will be not responsible if you lose your data. __Follow instructions carefully and make backups before you start!__</span> <br />
*For absolute beginners, we recommend going for any one of the three alternatives mentioned below unless you're sure you want to dual-boot. Dual-booting can be a little daunting. And you can always opt for dual-booting once you're comfortable with linux.* <br/>

	**Virtual Machine** : You can install a virtual machine and install ubuntu on that. Follow this [Tutorial](https://youtu.be/v1JVqd8M3Yc?si=AaufZwAfmLKAP4BB) in that case. (Installation of the Virtual Machine is also included in the tutorial)<br /><br />
	**WSL** : For Windows 11, check this [Tutorial](https://www.howtogeek.com/744328/how-to-install-the-windows-subsystem-for-linux-on-windows-11/) to set up WSL. <br /> <br />

  **Microsoft  Store** : You can also directly download the Ubuntu from the store, you will get a terminal.

	**The Construct website** : You are also free to use [**theconstructsim**](https://www.theconstructsim.com/) without having to install anything. The Construct is an online platform that supports ROS development.

</ul>

* __Get familiar with Linux__:
Here are a few additional resources that you can refer to in order to get familiar with Linux:
	* [Video-based Tutorial](https://www.youtube.com/watch?v=IVquJh3DXUA "Introduction to Linux and Basic Linux Commands for Beginners")
	* [Text-based Tutorial](https://ryanstutorials.net/linuxtutorial/ "Linux Tutorial")
  * [Document containing useful linux command lines](https://docs.google.com/document/d/1aroDJBIP-mqYovI8sVYYjGrn_1ugpN5NBauLLihvEjM/edit?usp=sharing)

* __Terminator installation for Ubuntu__:

	It's highly recommended to use Terminator instead of stock Terminal as you can have tabs or split windows into few terminals, which will be very useful throughout.

	To install **Terminator**, run the following command in the terminal:  
	```bash
	sudo apt-get install terminator
	```

* __ROS Installation/setup__:
	- For Ubuntu 22.04: [ROS2 HUMBLE](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
Go to a particular link and put your first step in the world of ROS.

      If you don’t want to have to source the setup file every time you open a new shell, then you can add the command to your shell startup script:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

* __IDE Installation__:

	You are free to use a suitabe IDE to write code. The most commonly used IDE is **Visual Studio Code**. You can install it in your Ubuntu system and install **ROS VSCode Extention** in the VSCode application.


## **Getting started with ROS:**<a name="Getting"></a>

*Now that the installation is done, let’s dive into ROS!*

### **What is ROS?**

ROS is a software framework for writing robot software. The main aim of ROS is to reuse the robotic software across the globe. ROS consists of a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.
The official definition on ROS wiki is:

*ROS is an open-source, meta-operating system for your robot. It provides the services you would expect from an operating system, including hardware abstraction, low-level device control, implementation of commonly-used functionality, message-passing between processes, and package management. It also provides tools and libraries for obtaining, building, writing, and running code across multiple computers. ROS is similar in some respects to robot frameworks, such as Player, YARP, Orocos, CARMEN, Orca, MOOS, and Microsoft Robotics Studio.*

### **Basics of ROS**

First of all, let us start with the basics of ROS.
It would be better if you **write the code on your own instead of copying and pasting it directly.** You will grasp the topics covered better when you try the implementation on your own. It's preferable if you use __Python__ instead of __C++__ as python syntax is easier and more readable.

To start off, these two tutorials will cover aspects such as **creating a ROS Workspace** and
[**navigating the ROS Filesystem**](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html). 
To help you a little bit more, we will also explain you some command line tools and will highlight the sections you should focus more when you open these links......

The bullet points below a particular link will help you focus more on the important stuff and will help you to navigate better....

Sections to focus in "Configuring your ROS environment":

- Sourcing of setup.bash file : "source /opt/ros/distro/setup.bash" 
The purpose of this command line is basically to tell your terminal the location of your workspace so that ROS can find its path. If we unsource the setup.bash files then the Ubuntu will not be able to locate the packages of that workspace.
- Looking at the video below will help more.
- Its an advise to not panic if you don't grasp things in one go, try to go completely by this twice or thrice for more clarity and try to connect dots while reading the second or third time.



Next, we shall look at packages.

#### **What is a package?** 

ROS uses **packages** to organize its programs. Every ROS program that you want to create or execute is organized in a package. You can think of a package as **all the files that a specific ROS program contains**; all its CPP files, python files, configuration files, compilation files, launch files, and parameter files. In ROS2 framework the script within a package can be either of Python or CPP strictly. We will follow Python. All those files in the package are organized with the following structure:

* __launch__ folder: Contains launch files
* __scripts folder__: Source files (Python)
* __package.xml__: Package information and dependencies

### **Colcon**

In the ROS ecosystem the software is separated into numerous packages. It is very common that a developer is working on multiple packages at the same time. This is in contrast to workflows where a developer only works on a single software package at a time, and all dependencies are provided once but not being iterated on. A build tool performs the task of building a set of packages with a single invocation.

For installing and configuring Colcon run the command:

```bash
sudo apt install python3-colcon-common-extensions
```

### **Create a workspace**

A workspace is a directory containing ROS 2 packages. Before using ROS 2, it’s necessary to source your ROS 2 installation workspace in the terminal you plan to work in. This makes ROS 2’s packages available for you to use in that terminal.

Best practice is to create a new directory for every new workspace.

```bash
mkdir -p ~/erc_ws/src
cd ~/erc_ws/src
```

From the root of your workspace (erc_ws), you can now build your packages using the command:

```bash
cd ..
colcon build
```

Now, let’s add the erc_ws path. Execute the following command:

```bash
cd ~		# go to the home folder
nano .bashrc	# open the .bashrc file
```
Add the command `source ~/erc_ws/install/setup.bash` to the end of *.bashrc*.  
Then, hit <kbd>CTRL</kbd>+<kbd>X</kbd>, then, <kbd>Y</kbd>, to save the changes to the file.

When you make changes to your .bashrc file, run:
```bash
source ~/.bashrc
```
This command applies the changes immediately without restarting your terminal.


### **Create a new package**

A package is an organizational unit for your ROS 2 code. If you want to be able to install your code or share it with others, then you’ll need it to be organized in a package. With packages, you can release your ROS 2 work and allow others to build and use it easily.

Package creation in ROS 2 uses ament as its build system and colcon as its build tool. You can create a package using either CMake or Python.

Recall that packages should be created in the src directory, not the root of the workspace. So, navigate into ```erc_ws/src```, and run the package creation command:

```bash
cd erc_ws/src
ros2 pkg create --build-type ament_python pubsub
```

If you get an error setup.py install is deprecated on running colcon build then you need to downgrade the setuptools to 58.2.0

```bash
pip3 install setuptools==58.2.0
```

### Nodes
One of the primary purposes of ROS is to facilitate communication between the ROS nodes. Every program in ROS is called a **node**. Every independent task can be separated into nodes which communicate with each other through channels. These channels are also known as **topics**.

For example, one node can capture the images from a camera and send the images to another node for processing. After processing the image, the second node can send a control signal to a third node for controlling a robotic manipulator in response to the camera view.

The main mechanism used by ROS nodes to communicate is by sending and receiving **messages**. The **messages** are organized into specific categories called **topics**. Nodes may **publish** messages on a particular topic or **subscribe** to a topic to receive information.

### Introducing TurtleSim

To demonstrate how to run nodes, let us run 'turtlesim_node' node from a pre-installed package, 'turtlesim':

To run the 'turtlesim_node' node, run this **in a terminal**:
```bash
ros2 run turtlesim turtlesim_node
```
You'll see the new turtlesim window.

## Using roslaunch to run multiple nodes at once

ROS 2 Launch files allow you to start up and configure a number of executables containing ROS 2 nodes simultaneously.

Create a new directory in erc_ws to store your launch files:
```bash
mkdir launch
```

Let’s put together a ROS 2 launch file using the turtlesim package and its executables.
Copy and paste the complete code into the ```launch/turtlesim_launch.py``` file:

```bash
touch turtlesim_launch.py
```

```bash
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        )
    ])
```

To run the launch file created above, enter into the directory you created earlier and run the following command:

```bash
cd launch
ros2 launch turtlesim_launch.py
```



### Topics

A topic is simply a medium of exchange of data (like a channel). Some nodes called **Publishers** can publish data on the topic, some nodes called **Subscribers** can subscribe to the data on the topic.

A topic has a message type (similar to the data type of a variable). All publishers and subscribers on this topic must publish/subscribe data of the associated message type.

You can create a publisher or subscriber in any ROS supported language you want, directly inside ROS nodes.

When a node wants to publish something, it will inform the ROS master. When another node wants to subscribe to a topic, it will ask the ROS master form where it can get the data.

Finally, a node can contain many publishers and subscribers for many different topics.

PS:

- ```rqt_graph```:
Reveals communication between nodes through topics.
Essential for understanding node interactions.

- ```ros2 topic echo```:
Displays real-time data published on a specific topic.
Useful for monitoring topic content.

- ```ros2 topic pub```:
Enables manual publication on desired topics.
Handy for testing and injecting data into the system.

Try running these commands and viewing the output.

## Publisher-Subscriber Interface <a name="PubSub"></a>

Message passing in ROS happens with the Publisher-Subscriber Interface provided by ROS library functions.

Creating a publisher or subscriber node is just like creating any other node. <br />

1. Go to the package where you want to create these nodes 

2. Make a new directory or folder 
 
3. Create python script files for a publisher and a subscriber


### Create a executeable python file

Navigate into ```erc_ws/src/pubsub/pubsub``` and then create a python file 

```bash
touch talker.py
chmod +x talker.py #Making the python file executable
```

### Writing a simple Publisher Node <a name="Publisher"></a>

This is a basic publisher node python script ```talker.py```  (taken from the official ROS tutorials from the website, and comments are added to help you understand the working of each line):

```python
#!/usr/bin/env python
import rclpy
from std_msgs.msg import String

def timer_callback(timer, i):
    # Create a String message and set its data
    msg = String()
    msg.data = 'Hello World: %d' % i
    
    # Publish the message
    publisher.publish(msg)
    
    # Log the published message
    get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    # Initialize the ROS2 client library
    rclpy.init(args=args)

    # Create a ROS2 node
    node = rclpy.create_node('minimal_publisher')

    # Create a publisher for the 'topic' with String messages
    global publisher
    publisher = node.create_publisher(String, 'topic', 10)

    # Get the logger for the node
    global get_logger
    get_logger = node.get_logger()

    # Set the timer period and initialize a counter
    timer_period = 0.5  # seconds
    i = 0

    # Create a timer and associate it with the timer_callback function
    timer = node.create_timer(timer_period, lambda timer: timer_callback(timer, i))

    # Spin the node to handle callbacks
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    # Shutdown the ROS2 client library
    rclpy.shutdown()

if _name_ == '_main_':
    main()
```

## Add dependencies
Navigate one level back to the ```erc_ws/src/pubsub``` directory, where the setup.py, setup.cfg, and package.xml files have been created for you.

Open package.xml with your text editor. Add the following dependencies corresponding to your node’s import statements:

```bash
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

This declares the package needs rclpy and std_msgs when its code is executed.


## Add an entry point

Open the setup.py file, and add the following line within the console_scripts brackets of the entry_points field:

```bash
entry_points={
        'console_scripts': [
                'publisher = pubsub.talker:main',
        ],
},
```


### Writing a simple Subscriber Node <a name="Subscriber"></a>

Make the listener.py file similarly as we have done for talker.py.

This is a basic subscriber node python script ```listener.py``` (taken from the official ROS tutorials from the website, and comments are added to help you understand the working of each line):

```python
#!/usr/bin/env python
import rclpy
from std_msgs.msg import String

def listener_callback(msg):
    # Log the received message
    get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    # Initialize the ROS2 client library
    rclpy.init(args=args)

    # Create a ROS2 node
    node = rclpy.create_node('minimal_subscriber')

    # Create a subscription for the 'topic' with String messages
    subscription = node.create_subscription(String, 'topic', listener_callback, 10)

    # Get the logger for the node
    get_logger = node.get_logger()

    # Spin the node to handle callbacks
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()

    # Shutdown the ROS2 client library
    rclpy.shutdown()

if _name_ == '_main_':
    main()
```

Now similarly for subscriber add entry points as in publisher node.

```bash
entry_points={
        'console_scripts': [
                'publisher = pubsub.talker:main',
                'subscriber = pubsub.listener:main',
        ],
},
```

## Build and run

You likely already have the ```rclpy``` and ```std_msgs``` packages installed as part of your ROS 2 system. It’s good practice to run rosdep in the root of your workspace (erc_ws) to check for missing dependencies before building:

```bash
rosdep install -i --from-path src --rosdistro humble -y
```
	
Finally, go to erc_ws and build the package

```python
colcon build
```

Now run the publisher node:

```bash
ros2 run pubsub publisher
```

Similarly run the subscriber node in a new terminal. Remember to source the workspace if you haven't already.

```bash
ros2 run pubsub subscriber
```

You can see that 'Hello World: n' is being printed. The Publisher Node is up and running!

You can see that 'I heard: "Hello World: n' is being printed. The Subscriber Node is running as well.

Note that once you stop running the Publisher Node ( Press `Ctrl`+`C` while you're in the terminal that is running the Publisher Node), the Subscriber Node stops running as well. 

### Running the publisher and subscriber using a launch file

Create a new directory in your package to store your launch files
```bash
mkdir launch
```
Create a file ```pubsub.py``` in the ```launch``` folder of ```erc_ws``` 

```bash
touch pubsub.py
```

Add the following code.

```bash
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pubsub',
            executable='talker.py',
            name='publisher'
        ),
        Node(
            package='pubsub',
            executable='listener.py',
            name='subscriber'
        ),
    ])
```

```name``` refers to the name of the node, ```package``` refers to the name of the package in which the node is present in, ```executable``` is the name of the node file

On executing ```ros2 launch pubsub.py```, you will be able to see **Publisher** and **Subscriber** in the list of Nodes. 

While the system is still running, open a new terminal and run ```rqt_graph``` to get a better idea of the relationship between the nodes in your launch file.

## Now, enough chatter. Time to do ...

Create a new package ```sherlock``` in ```erc_ws```, which will contain three nodes and a launch file.

1) The first node will publish the following text to the topic ```listen_1```.

	**I am not a psychopath, Anderson.**

2) The second node will publish the following text to the topic ```listen_2```.

 	**I am a high-functioning sociopath.**

3) The third node will subscribe to ```listen_1``` and ```listen_2```.

4) The third node should display the following statement on the terminal at some frequency.

	**I am not a psychopath, Anderson. I am a high-functioning sociopath.**

5) The launch file will launch all the three nodes.

<img src="W0_Images/Sociopath.gif" width=400 height=220>

Have fun !