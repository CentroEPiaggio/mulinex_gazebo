import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  use_sim_time = LaunchConfiguration('use_sim_time', default='false')
  urdf = os.path.join(
        get_package_share_directory('mulinex_description'),
        'urdf',
        'mulinex.xacro'
  )
  print("urdf_file_name : {}".format(urdf))


  return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
            {'robot_description': Command(['xacro', ' ', urdf, ' use_gazebo:=true ',
                        ' front_hfe:=-2.094', 
                        ' hind_hfe:=2.094',
                        ' front_kfe:=0.7854',
                        ' hind_kfe:=-0.7854',])}],
            ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
            ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "mulinex",
                        "-x", "0", "-y", "0", "-z", "0.5",]),

        IncludeLaunchDescription(
          PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rlg_quad_controller'),
            'launch',
            'mulinex_inference.launch.py')])
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, 
                        os.path.join(
                            get_package_share_directory('mulinex_gazebo'),
                            'config',
                            'my_robot_controllers.yaml')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
        ),
  ])