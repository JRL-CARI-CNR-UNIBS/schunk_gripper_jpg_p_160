# Copyright 2023 Áron Svastits
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):

    # args that can be set from the command line or a default will be used
    prefix = LaunchConfiguration("prefix")

    rviz_config_file = PathJoinSubstitution([FindPackageShare('schunk_gripper_jpg_p_160_description'),
                                            "config", "view_cell.rviz"]
                                            )

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare('schunk_gripper_jpg_p_160_description'),
                     "urdf", "schunk_gripper_jpg_p_160_rviz.xacro"]
                ),
                " ",
                "prefix:=",
                prefix.perform(context),
            ]
        ),
        value_type=str
    )

    print(type(robot_description_content))
    robot_description = {'robot_description': robot_description_content}

    joint_target_publisher = Node(
        name='joint_target_publisher',
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'rate': 500}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        joint_target_publisher,
        robot_state_publisher,
        rviz]

    return nodes_to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument(
        'prefix',
        default_value=''
    ))
    return LaunchDescription(launch_arguments +
                             [OpaqueFunction(function=launch_setup)])
