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

#!/bin/bash
# This file is for docker container
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source /sim_ws/install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
