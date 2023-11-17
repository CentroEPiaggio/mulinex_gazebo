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
            launch_arguments={"verbose": "false"}.items(),
    )

    # Get URDF via xacro
    mulinex_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ", urdf, " use_gazebo:=true ",
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
                   "-x", "0", "-y", "0", "-z", "0.5"],
        output="screen",
    )

    # control_manager_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     name = 'controller_manager',
    #     parameters=[robot_description],
    #     output="screen",
    # )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    PD_jnt_control = Node(
        package="controller_manager",
        executable="spawner",
        #arguments=['GazeboSystem'],
        arguments=["PD_control", "--controller-manager", "/controller_manager"],
    )


    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        # IncludeLaunchDescription(
        #   PythonLaunchDescriptionSource([os.path.join(
        #     get_package_share_directory('rlg_quad_controller'),
        #     'launch',
        #     'mulinex_inference.launch.py')])
        # ),
        gazebo,
        # control_manager_node,
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