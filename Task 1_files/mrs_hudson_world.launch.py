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
	
	pkg_project = get_package_share_directory('task1')
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
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': os.path.join(pkg_project, 'worlds', 'mrs_hudson_arena.sdf')}.items(),
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
        
	rviz = Node(
	package='rviz2',
	executable='rviz2',
	arguments=['-d', os.path.join(pkg_project, 'config', 'mrs_hudson.rviz')],
	)        

	return LaunchDescription([
		robot_state_publisher,
		rviz,	            
		gz_sim,
		bridge
		])
