"""Launch the G1 muscle-up (update 2)

   ros2 launch muscleup_update1 muscleup.launch.py
   ros2 launch muscleup_update1 muscleup.launch.py motion:=low  target:=-1.0
   ros2 launch muscleup_update1 muscleup.launch.py motion:=side target:=0.40
   ros2 launch muscleup_update1 muscleup.launch.py motion:=low  target:=-1.0 priority:=false

This starts
  1) RVIZ, ready to view the robot and bar
  2) The robot_state_publisher to broadcast the G1 model
  3) The muscleup node (trajectory and inverse kinematics)
"""

import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import DeclareLaunchArgument, Shutdown
from launch.substitutions              import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions                import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = pkgdir('muscleup_update1')
    with open(os.path.join(share, 'urdf', 'g1_29dof_hands.urdf'), 'r') as file:
        robot_description = file.read()

    motion = DeclareLaunchArgument('motion', default_value='muscleup',
                                   description='muscleup, low, or side')
    target = DeclareLaunchArgument('target', default_value='0.0',
                                   description='low: pelvis z (m); side: pelvis y (m)')
    priority = DeclareLaunchArgument('priority', default_value='true',
                                   description='false = update 1 equal weights')
    view = DeclareLaunchArgument('view', default_value='muscleup_side.rviz',
                                 description='RViz config in rviz/ (muscleup.rviz = front)')

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
        arguments  = ['-d', PathJoinSubstitution([share, 'rviz', LaunchConfiguration('view')])],
        on_exit    = Shutdown())

    node_muscleup = Node(
        name       = 'muscleup',
        package    = 'muscleup_update1',
        executable = 'muscleup',
        output     = 'screen',
        parameters = [{'motion': LaunchConfiguration('motion'),
                       'target': ParameterValue(LaunchConfiguration('target'),
                                                value_type=float),
                       'priority': ParameterValue(LaunchConfiguration('priority'),
                                                  value_type=bool)}])

    return LaunchDescription([
        motion,
        target,
        priority,
        view,
        node_robot_state_publisher,
        node_rviz,
        node_muscleup,
    ])
