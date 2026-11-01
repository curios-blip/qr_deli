import os
from ament_index_python.packages import get_package_share_directory as pkg
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource as Src

def generate_launch_description():
    tb3 = os.path.join(pkg('turtlebot3_gazebo'), 'launch')
    gz = os.path.join(pkg('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    world = os.path.expanduser('~/qr_delivery/worlds/rooms.sdf')
    return LaunchDescription([
        IncludeLaunchDescription(Src(gz), launch_arguments={'gz_args': f'-r -s -v2 {world}', 'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(Src(gz), launch_arguments={'gz_args': '-g -v2'}.items()),
        IncludeLaunchDescription(Src(os.path.join(tb3, 'robot_state_publisher.launch.py')), launch_arguments={'use_sim_time': 'true'}.items()),
        IncludeLaunchDescription(Src(os.path.join(tb3, 'spawn_turtlebot3.launch.py')), launch_arguments={'x_pose': '0.7', 'y_pose': '0.0'}.items()),
    ])
