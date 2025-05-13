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

import rclpy, numpy as np, math, time, yaml, os
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory

MAX_SPEED = 1.5
MIN_SPEED = 1.0
ANGLE_GAIN = 1.7
SAFE_GAP = 2.0          # m
BUBBLE_RADIUS = 0.3     # m
FOV = math.radians(270) # the lidar in gym is 270 degrees

def best_gap(ranges, angle_min, angle_inc):
    arr = np.array(ranges)
    arr[arr < BUBBLE_RADIUS] = 0.0
    mask = arr > SAFE_GAP
    if not mask.any():
        return 0.0
    segs = np.where(np.diff(mask.astype(int)) != 0)[0] + 1
    indices = np.split(np.arange(arr.size), segs)
    gaps = [idx for idx in indices if mask[idx].any()]
    best = max(gaps, key=len)
    mid = best[len(best)//2]
    return angle_min + mid * angle_inc

class GapFollower(Node):
    def __init__(self, ns):
        super().__init__(f'{ns}_gap_follower')
        self.ns = ns
        self.drive_pub = self.create_publisher(AckermannDriveStamped, f'/{ns}/drive', 1)
        self.create_subscription(LaserScan, f'/{ns}/scan', self.cb, 1)

    def cb(self, scan):
        steer = best_gap(scan.ranges, scan.angle_min, scan.angle_increment)
        speed = max(MIN_SPEED, MAX_SPEED * (1 - abs(steer)/ (FOV/2)))
        msg = AckermannDriveStamped()
        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = f"{self.ns}/base_link"
        msg.drive.steering_angle = steer * ANGLE_GAIN
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

def main():
    rclpy.init()

    sim_config = os.path.join(
        get_package_share_directory("f1tenth_gym_ros"), "config", "sim.yaml"
    )

    config_dict = yaml.safe_load(open(sim_config, "r"))
    num_agents = config_dict["gym_bridge"]["ros__parameters"]["num_agent"]
    ns_list = [f'racecar{i+1}' for i in range(num_agents)]
    exec_ = rclpy.executors.MultiThreadedExecutor()
    nodes = [GapFollower(ns) for ns in ns_list]
    
    for n in nodes:
        exec_.add_node(n)

    try:
        exec_.spin()
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
