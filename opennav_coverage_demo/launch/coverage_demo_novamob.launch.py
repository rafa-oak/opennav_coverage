# Copyright (c) 2023 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable,  RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, PythonExpression, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    coverage_demo_dir = get_package_share_directory('opennav_coverage_demo')
    # Get the path to the scout_v2.xacro file
    scout_nav2_gz_path = get_package_share_directory('novamob_nav2_gz')

    default_world_path = os.path.join(coverage_demo_dir, 'empty.sdf')
    param_file_path = os.path.join(coverage_demo_dir, 'demo_params_novamob.yaml')

    default_model_path = os.path.join(scout_nav2_gz_path, "src/description/novamob_description.urdf")
    trailer_model_path = os.path.join(scout_nav2_gz_path, "src/description/novamob_trailer_description.urdf")
    gz_models_path = os.path.join(scout_nav2_gz_path, "models")


    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_trailer = LaunchConfiguration("use_trailer")
    log_level = LaunchConfiguration("log_level")
    gz_verbosity = LaunchConfiguration("gz_verbosity")
    run_headless = LaunchConfiguration("run_headless")
    world_path = LaunchConfiguration("world")  
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")


    gz_env = {'GZ_SIM_SYSTEM_PLUGIN_PATH':
           ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                     os.environ.get('LD_LIBRARY_PATH', default='')]),
           'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':  # TODO(CH3): To support pre-garden. Deprecated.
                      ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                                os.environ.get('LD_LIBRARY_PATH', default='')])}
    gazebo = [
        ExecuteProcess(
            condition=launch.conditions.IfCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, '-s', '--headless-rendering', world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        ),
        ExecuteProcess(
            condition=launch.conditions.UnlessCondition(run_headless),
            cmd=['ruby', FindExecutable(name="ign"), 'gazebo',  '-r', '-v', gz_verbosity, world_path],
            output='screen',
            additional_env=gz_env, # type: ignore
            shell=False,
        )
    ]

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name",
            "novamob",
            "-topic",
            "robot_description",
            '-x', '-10', '-y', '-10', '-z', '1.00',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
            "--ros-args",
            "--log-level",
            log_level,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/depth_camera@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/robot_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
            # Clock message is necessary for the diff_drive_controller to accept commands https://github.com/ros-controls/gz_ros2_control/issues/106
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        remappings=[
            ("/navsat", "/gps/fix")  # Remapping the Ignition GPS topic to the ROS /gps/fix topic
        ],
        output="screen",
    )


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": Command([
                    "xacro ",
                    PythonExpression([
                        "'", trailer_model_path, "' if '", LaunchConfiguration("use_trailer"), "' == 'True' else '", LaunchConfiguration("model"), "'"
                    ])
                ])
            }
        ],
    )
    # start the visualization
    rviz_config = os.path.join(coverage_demo_dir, 'opennav_coverage_demo.rviz')
    # print("rviz_config:", rviz_config)
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '', 'rviz_config': rviz_config}.items())

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(coverage_demo_dir, 'bringup_scout.launch.py')),
        launch_arguments={'params_file': param_file_path}.items())

    # world->odom transform, no localization. For visualization & controller transform
    fake_localization_cmd = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-10', '-10', '0', '0', '0', '0', 'map', 'odom'])

    # start the demo task
    demo_cmd = Node(
        package='opennav_coverage_demo',
        executable='demo_coverage_custom',
        emulate_tty=True,
        output='screen')

    relay_odom = Node(
        name="relay_odom",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/diff_drive_base_controller/odom",
                "output_topic": "/odom",
            }
        ],
        output="screen",
    )

    load_joint_state_controller = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        shell=False,
        output="screen",
    )

    load_joint_trajectory_controller = ExecuteProcess(
        name="activate_diff_drive_base_controller",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "diff_drive_base_controller",
        ],
        shell=False,
        output="screen",
    )

    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diff_drive_base_controller/cmd_vel_unstamped",
            }
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=gz_models_path,
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_MODEL_PATH",
                value=gz_models_path,
            ),
            DeclareLaunchArgument(
                name="model",
                default_value=default_model_path,
                description="Absolute path to robot urdf file",
            ),
            DeclareLaunchArgument(
                name="use_trailer", 
                default_value="False",
                description="Use the robot model with a trailer",
            ),
            DeclareLaunchArgument(
                name="use_rviz",
                default_value="True",
                description="Start RViz",
            ),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="Start GZ in hedless mode and don't start RViz (overrides use_rviz)",
            ),
            DeclareLaunchArgument(
                name="rvizconfig",
                default_value=rviz_config,
                description="Absolute path to rviz config file",
            ),
            DeclareLaunchArgument(
                name="use_sim_time",
                default_value="True",
                description="Flag to enable use_sim_time",
            ),
            DeclareLaunchArgument(
                name="use_localization",
                default_value="True",
                description="Use EKF to estimagte odom->base_link transform from IMU + wheel odometry",
            ),
            DeclareLaunchArgument(
                "gz_verbosity",
                default_value="3",
                description="Verbosity level for Ignition Gazebo (0~4).",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="",
                description="Extra args for Gazebo (ie. '-s' for running headless)",
            ),
            DeclareLaunchArgument(
                name="log_level",
                default_value="warn",
                description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
            ),
            DeclareLaunchArgument(
                name="world",
                default_value=default_world_path,  
                description="Absolute path to the world file to load in Ignition Gazebo",
            ),
            bridge,
            robot_state_publisher_node,
            spawn_entity,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_entity,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[load_joint_trajectory_controller],
                )
            ),
            rviz_cmd,
            bringup_cmd,
            fake_localization_cmd,
            demo_cmd,
            relay_odom,
            relay_cmd_vel,            
        ] + gazebo
    )