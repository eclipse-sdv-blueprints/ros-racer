# MIT License

# Copyright (c) 2020 Hongrui Zheng
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#
# Copyright (c) 2025 Composiv.ai
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0.
#
# SPDX-License-Identifier: EPL-2.0
#
# Contributors:
#   Composiv.ai - initial API and implementation
#


import os
import yaml
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    sim_config = os.path.join(
        get_package_share_directory("f1tenth_gym_ros"), "config", "sim.yaml"
    )

    config_dict = yaml.safe_load(open(sim_config, "r"))
    num_agents = config_dict["gym_bridge"]["ros__parameters"]["num_agent"]
    map_path = config_dict["gym_bridge"]["ros__parameters"]["map_path"]
    default_map_path = os.path.join(get_package_share_directory("f1tenth_gym_ros"),
                                    "maps",
                                    "Spielberg_map")
    
    print(f"Starting with {num_agents} agents.")
    gym_bridge_node = Node(
        package="f1tenth_gym_ros",
        executable="gym_bridge",
        name="gym_bridge",
        parameters=[sim_config],
    )

    racecar_config = os.path.join(
        get_package_share_directory("f1tenth_gym_ros"), "config", "racecar.yaml"
    )
    racecar_config_dict = yaml.safe_load(open(racecar_config, "r"))
    robot_state_publishers = []
    

    for i in range(num_agents):
        vehicle_name = racecar_config_dict["racecar"][f"agent{i+1}"]["vehicle_name"]
        color = racecar_config_dict["racecar"][f"agent{i+1}"]["color"]
        robot_state_publishers.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name=f"{vehicle_name}_robot_description",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                "xacro ",
                                os.path.join(
                                    get_package_share_directory('f1tenth_gym_ros'),
                                    "launch",
                                    "ego_racecar.xacro",
                                ),
                                f" car:={vehicle_name}",
                                f" car_color:={color}"
                            ]
                        )
                    }
                ],
                remappings=[
                    ("/robot_description", f"/{vehicle_name}/robot_description")],
            )
        )

    rosbridge_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('rosbridge_server'),
                     'launch',
                     'rosbridge_websocket_launch.xml')
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("f1tenth_gym_ros"),
                "launch",
                "gym_bridge.rviz",
            ),
        ],
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        parameters=[
            {
                "yaml_filename": map_path + ".yaml" if map_path else default_map_path + ".yaml"
            },
            {"topic": "map"},
            {"frame_id": "map"},
            {"output": "screen"},
            {"use_sim_time": True},
        ],
    )

    nav_lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    gap_follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'gap_follow.launch.py'))
    )
    ld.add_action(gap_follow)


    # finalize
    ld.add_action(rviz_node)
    ld.add_action(gym_bridge_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    for i in robot_state_publishers:
        ld.add_action(i)
    # ld.add_action(rosbridge_launch)
    return ld
