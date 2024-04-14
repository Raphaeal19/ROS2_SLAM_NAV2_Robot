import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node



def generate_launch_description():

    use_ros2_control = LaunchConfiguration('use_ros2_control')


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='my_bot' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': use_ros2_control}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file' + gazebo_params_file}.items(),
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot'],
                        output='screen')

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"]
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"]
    )

    keyboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'keyboard.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time':True}],
        remappings=[('/cmd_vel_out', 'diff_cont/cmd_vel_unstamped')]
    )

    online_async_params = os.path.join(get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml')
    online_async = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'online_async_launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'slam_params_file':online_async_params}.items()
    )

    navigation_params = os.path.join(get_package_share_directory(package_name), 'config', 'nav2_params.yaml')
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'navigation_launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'params_file': navigation_params}.items()
    )





    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control if true'),
        rsp,
        gazebo, 
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        keyboard,
        twist_mux,
        online_async,
        navigation,
    ])