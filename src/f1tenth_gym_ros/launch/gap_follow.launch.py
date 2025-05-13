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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    sim_config = os.path.join(
        get_package_share_directory("f1tenth_gym_ros"), "config", "sim.yaml"
    )

    config_dict = yaml.safe_load(open(sim_config, "r"))
    num_agents = config_dict["gym_bridge"]["ros__parameters"]["num_agent"]

    ld = LaunchDescription()

    for i in range(num_agents):
        ns = f'racecar{i+1}'
        ld.add_action(Node(package='f1tenth_gym_ros', executable='gap_follower', name=f'{ns}_gap', namespace='', parameters=[]))
    return ld
