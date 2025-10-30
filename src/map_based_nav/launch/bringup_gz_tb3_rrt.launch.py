from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('map_based_nav')
    world = PathJoinSubstitution([pkg_share, 'worlds', 'my_world.sdf'])
    tb3_sdf = PathJoinSubstitution([pkg_share, 'models', 'tb3_gz.sdf'])

    # 1) GZ: abre o mundo
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world],
        output='screen'
    )

    # 2) GZ: spawn do TB3 (SDF com plugin diff-drive)
    spawn_tb3 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', tb3_sdf, '-name', 'tb3'],
        output='screen'
    )

    # 3) Bridge ROS<->GZ (remaps para /cmd_vel e /odom)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/tb3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/model/tb3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '--ros-args',
            '-r', '/model/tb3/cmd_vel:=/cmd_vel',
            '-r', '/model/tb3/odometry:=/odom',
        ],
        output='screen'
    )

    # 4) Planner RRT
    planner = Node(
        package='map_based_nav',
        executable='rrt_planner',
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        spawn_tb3,
        bridge,
        planner
    ])
