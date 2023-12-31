## Episode 3 - The Final Showdown

Welcome to the final episode of Frosty Winter!

In this week, we first focus on an example of autonomous navigation and then look at the various applications ROS2 can have, along with some pointers to resources to help you look beyond this workshop on your journey in robotics.

To start off, we look at an example of autonomous navigation in a grid. You might have seen some naive methods for this while doing the first task. As usual, we will be using our own mrs_hudson as the bot for exploring this weeks content. Compared to earlier weeks, this week would be lighter, and we will only be tying+- up any loose ends from previous weeks.


### Navigation in a Closed Maze 

First, create a new package in the src directory of your erc_ws named ep3, with dependencies on rclpy, std_msgs and gazebo_ros. In this package, create folders for launch, script and worlds. Copy [this file](W3_Files/maze.world) and put this into the worlds folder, and copy [this file](W3_Files/maze_world.launch) and put it into the launch folder. This was just to load the map in gazebo and spawn the turtlebot.

To ensure everything is working, go to your erc_ws, and run ```colcon build``` . Then run

```
ros2 launch ep3 maze_world.launch
```
This should launch gazebo with a closed maze and mrs_hudson inside it.

Now, we shall write a script to automate the process of navigation, i.e. to keep travelling within this maze.

In ep3 folder, create a file named 'navigator.py'. We are going to create a class named AutoNavigator to handle this navigation for us. Looking into the code snippet by snippet,

```python
# Importing libraries

import rclpy
from geometry_msgs.msg import Twist    #For /cmd_vel
from sensor_msgs.msg import LaserScan  #For /scan

```

Now, to define the skeleton of the class and the main function:

```python
class AutoNavigator:
	def __init__(self):
		pass
	
	def scan_callback(self,msg):
		pass
	
	def run(self):
		pass

def main(args=None):
    navigator = AutoNavigator()
    navigator.run()
		
if __name__=='__main__':
	main()
```


First, we need to define the constructor. We need to add publishers to /cmd_vel topic, we need to subscribe to /scan topic. We also declare helper variables to act as a command to the /cmd_vel topic, and store the minimum distances in the front, left and right based on the callback from the /scan topic. We initialise all these variables in the constructor.

```python
def __init__(self):
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.command = Twist()
        self.command.linear.x = 0
        self.command.angular.z = 0
        self.near_wall = False
        self.min_front = 1
        self.min_right = 1
        self.min_left = 1
        self.min_range = 1
        self.rate = self.create_rate(10)
 ```


We then proceed to write the scan_callback function. The ranges variable in the message returned by laser scan contains a list of distances of objects with indices based on the relative angle of the point with respect to the front of the robot. For frontal distances, we take distances between -5 and 5 degrees, for the rightside we consider distances between 300 and 345 degrees, and for left we consider between 15 and 60 degrees. We then take the minimum of each of these distances and update them in the variables declared earlier.
```python
def scan_callback(self,msg):
    allranges = msg.ranges
    frontal = allranges[0:5] + allranges[-1:-5:-1]
    rightside = allranges[300:345]
    leftside = allranges[15:60]
    self.min_left = min(leftside)
    self.min_right = min(rightside)
    self.min_front = min(frontal)
    self.min_range = min( self.min_left , self.min_front , self.min_right )

```

Now, we need to define the run function of this class. The algorithm we're using for navigation is fairly simple:
* First we travel forward until we reach near a wall.
* If we reach a wall, then we start following the left side wall. 
* If it gets too close to the wall, we reverse a bit.
* Else, continue following the wall by reversing a little bit.
* If the front is too close to a wall, then turn until its clear.

The code below uses few parameters for the linear and angular velocities. Do feel free to change them and experiment with the results. 

```python
def run(self):
    while rclpy.ok():
    
    	#This following while loop runs only in the start while it is moving towards a wall after which near_wall becomes true, and is never reset.
        while self.near_wall==False and not rclpy.ok():
            print("Moving to wall")
            
            # If not near any wall, continue travelling
            if self.min_range > 0.2:
                self.command.angular.z = -0.1
                self.command.linear.x = 0.15
                
            # If wall on left side, break out of loop and follow left side
            elif self.min_left < 0.2:
                self.near_wall = True
                
            # Rotate until wall is on left
            else:
                self.command.angular.z = -0.25
                self.command.linear.x = 0

            self.cmd_vel_pub.publish(self.command)
            self.rate.sleep()
            
            
        #Once we have found the wall for the first time, this is always executed.
        else:
        
        	#If there is space in front of robot, we move in zig zag format by staying near left wall.
            if (self.min_front > 0.2):
                if (self.min_left < 0.12):
                    print("Range: {:.2f}m - Too close. Backing up.".format(self.min_left))
                    self.command.angular.z = -1.2
                    self.command.linear.x = -0.1
                    
                #If too far from wall, go near wall
                elif self.min_left > 0.15:
                    print("Range: {:.2f}m - Wall-following; turn left.".format(self.min_left))
                    self.command.angular.z = 1.2
                    self.command.linear.x = 0.15
                    
                # If not too far, then go away from wall
                else:
                    print("Range: {:.2f}m - Wall-following; turn right.".format(self.min_left))
                    self.command.angular.z = -1.2
                    self.command.linear.x = 0.15
                    
            # If there's an obstacle in the front, rotate clockwise until that obstacle becomes the left wall.
            else:
                print("Front obstacle detected. Turning away.")
                self.command.angular.z = -1.0
                self.command.linear.x = 0.0
                self.cmd_vel_pub.publish(self.command)
            self.cmd_vel_pub.publish(self.command)
        self.rate.sleep()
```



To summarise, your navigator.py will look like (without the comments added here to help you understand):

```python
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class AutoNavigator:
    def __init__(self):
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.command = Twist()
        self.command.linear.x = 0
        self.command.angular.z = 0
        self.near_wall = False
        self.min_front = 1
        self.min_right = 1
        self.min_left = 1
        self.min_range = 1
        self.rate = self.create_rate(10)

    def scan_callback(self, msg):
        all_ranges = msg.ranges
        frontal = all_ranges[0:5] + all_ranges[-1:-5:-1]
        right_side = all_ranges[300:345]
        left_side = all_ranges[15:60]
        self.min_left = min(left_side)
        self.min_right = min(right_side)
        self.min_front = min(frontal)
        self.min_range = min(self.min_left, self.min_front, self.min_right)

    def run(self):
        while rclpy.ok():
            while not self.near_wall and rclpy.ok():
                print("Moving to wall")
                if self.min_range > 0.2:
                    self.command.angular.z = -0.1
                    self.command.linear.x = 0.15
                elif self.min_left < 0.2:
                    self.near_wall = True
                else:
                    self.command.angular.z = -0.25
                    self.command.linear.x = 0

                self.cmd_vel_pub.publish(self.command)
                self.rate.sleep()
            else:
                if self.min_front > 0.2:
                    if self.min_left < 0.12:
                        print("Range: {:.2f}m - Too close. Backing up.".format(self.min_left))
                        self.command.angular.z = -1.2
                        self.command.linear.x = -0.1
                    elif self.min_left > 0.15:
                        print("Range: {:.2f}m - Wall-following; turn left.".format(self.min_left))
                        self.command.angular.z = 1.2
                        self.command.linear.x = 0.15
                    else:
                        print("Range: {:.2f}m - Wall-following; turn right.".format(self.min_left))
                        self.command.angular.z = -1.2
                        self.command.linear.x = 0.15
                else:
                    print("Front obstacle detected. Turning away.")
                    self.command.angular.z = -1.0
                    self.command.linear.x = 0.0
                    self.cmd_vel_pub.publish(self.command)
                self.cmd_vel_pub.publish(self.command)
            self.rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    navigator = AutoNavigator()
    navigator.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Now, to run this, execute

```
ros2 run ep3 navigator.py
```

The bot should start moving inside the maze following the walls.

This summarises one example of autonomous navigation (in this case using wall following). 

### Looking Ahead

Through this workshop, you were introduced to some tools used in Robotics centered around ROS2. You saw the basics of ROS2, simulation and visualization tools like RViz and Gazebo, image processing software - OpenCV, ArUco markers and many more. You also saw a basic autonomous navigation of a grid using the mrs_hudson by travelling along the walls of the grid. 

Before we proceed to the task for this week, we first present a few points that may or may not be of interest to you going ahead in ROS.

* In order to explore on how to proceed to use ROS in your own custom bots, you could refer to the 6th section of the ROS tutorials on the ROS wiki. (Can be found [here](http://wiki.ros.org/ROS/Tutorials))
* For anything else regarding ROS, your go-to site will mostly be the [ROS Wiki](http://wiki.ros.org/). It contains relevant resources pertaining to all aspects of ROS, such as Tutorials, ROS Community, various packages, how to contribute your own packages to ROS etc.


We now head towards the task for this week.

# Final Task

So finally we have reached towards the final task of the workshop.  

<img src="W3_Files/benedict-cumberbatch-oh.gif" width=500 height=500>

We hope you have gone through all the episodes and had a great time learning. If you have any doubts you can reach out to us.

In this task you have to write a code for autonomous navigation of bot to solve maze. As you saw in week2 ArUco will help you to solve maze.
Git pull the latest mrs_hudson package.
Open the Terminal and run following commands-
```bash
cd ~/erc_ws/src/mrs_hudson
git pull
cd ~/erc_ws
colcon build
```
<img src="W3_Files/final_maze.jpg" width=600 height=400>

## Some points to Remember

Run maze_aruco.launch.py file to launch final_maze.sdf.

Cool!!

In scripts folder, you will see one more file navigator.py 
We have given helper code in it, you have to edit this file to complete task2


## Submission details

### Tentative deadline
10 January 2022, 11:59 PM

### Mode of Submission

The submission link is attached below. 

[Submission Link]()