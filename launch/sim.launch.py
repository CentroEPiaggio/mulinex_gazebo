import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,RegisterEventHandler
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
import numpy as np

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(
        get_package_share_directory('mulinex_description'),
        'urdf',
        'mulinex.xacro'
    )

    print("urdf_file_name : {}".format(urdf))

    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])]
            ),
            launch_arguments={"pause": "true", "verbose": "false"}.items(),
    )

    default_dof    = (
            2.094,     
            -2.094,    
            -2.094,
            2.094,
            -0.7854,   
            0.7854,    
            0.7854,
            -0.7854,
    )

    hip_angle = 120.0
    knee_angle = 60.0

    default_dof    = (
            np.deg2rad(hip_angle),     
            -(np.deg2rad(hip_angle)),    
            -(np.deg2rad(hip_angle)),
            np.deg2rad(hip_angle),
            -np.deg2rad(knee_angle),   
            np.deg2rad(knee_angle),    
            np.deg2rad(knee_angle),
            -np.deg2rad(knee_angle),
    )

    joint_names=(
        'LF_HFE',   
        'LH_HFE',   
        'RF_HFE',
        'RH_HFE',
        'LF_KFE',   
        'LH_KFE',   
        'RF_KFE',
        'RH_KFE',
    )
    default_dict = dict(zip(joint_names, default_dof))

    #format is "NAME:=VALUE "
    default_joint_args = ""
    for key, value in default_dict.items():
        default_joint_args += key + ":=" + str(value) + " "

    # Get URDF via xacro
    mulinex_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", urdf, " use_gazebo:=true ",
            default_joint_args
        ]
    )

    robot_description = {"robot_description": mulinex_description_content}


    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "mulinex",
                    "-x", "0", "-y", "0", "-z", "0.4",
                    ],
        output="screen",
    )


    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    PD_jnt_control = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["PD_control", "--controller-manager", "/controller_manager"],
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        gazebo,

        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[PD_jnt_control],
            )
        ),
     
  ])