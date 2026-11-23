"""Launch the G1 pull-up + typewriter

   ros2 launch muscleup_update1 typewriter.launch.py
   ros2 launch muscleup_update1 typewriter.launch.py shift:=0.25

This starts
  1) RVIZ, ready to view the robot and bar
  2) The robot_state_publisher to broadcast the G1 model
  3) The typewriter node (trajectory and inverse kinematics)
"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import DeclareLaunchArgument, Shutdown
from launch.substitutions              import LaunchConfiguration
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = pkgdir('muscleup_update1')
    with open(os.path.join(share, 'urdf', 'g1_29dof_hands.urdf'), 'r') as file:
        robot_description = file.read()

    shift = DeclareLaunchArgument('shift', default_value='0.15',
                                  description='Pelvis shift to each side (m)')

    node_robot_state_publisher = Node(
        name       = 'robot_state_publisher',
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': robot_description}])

    node_rviz = Node(
        name       = 'rviz',
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', os.path.join(share, 'rviz', 'muscleup.rviz')],
        on_exit    = Shutdown())

    node_typewriter = Node(
        name       = 'typewriter',
        package    = 'muscleup_update1',
        executable = 'typewriter',
        output     = 'screen',
        parameters = [{'shift': ParameterValue(LaunchConfiguration('shift'),
                                               value_type=float)}])

    return LaunchDescription([
        shift,
        node_robot_state_publisher,
        node_rviz,
        node_typewriter,
    ])
