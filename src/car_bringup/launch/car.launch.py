import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from ament_index_python import get_package_share_path

def generate_launch_description():
    
    car_description_path = get_package_share_path('car_description')
    car_bringup_path = get_package_share_path('car_bringup')
    
    xacro_file = os.path.join(car_description_path, 'xacro', 'main.xacro')
    rviz_config_file = os.path.join(car_bringup_path, 'rviz', 'xacro_config.rviz')
    
   

    return LaunchDescription([
       
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', xacro_file])}]
        ),

      
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot_v2']
        ),

     
        ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', 
                 '--verbose', 'worlds/empty.world'],
            output='screen'
        ),


        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),


        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(rviz_config_file)]
        ),


        Node(
            package='car_py_pkg',
            executable='movement_node',
            output='screen'
        ),
        

        Node(
            package='car_py_pkg',
            executable='gps_xyz_node',
            output='screen'
        ),


        Node(
            package='car_py_pkg',
            executable='imu_monitor_node',
            output='screen'
        )
            
    ])