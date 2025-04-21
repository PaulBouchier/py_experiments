#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
Basic navigation demo to go to pose.
"""

def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    goal1_pose = PoseStamped()
    goal1_pose.header.frame_id = 'map'
    goal1_pose.header.stamp = navigator.get_clock().now().to_msg()

    goal1_pose.pose.position.x = 1.0
    goal1_pose.pose.position.y = 1.0
    goal1_pose.pose.orientation.w = 1.0
    goal1_pose.pose.orientation.z = 0.0

    navigator.goToPose(goal1_pose)

    try:
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
    except KeyboardInterrupt:
        print('Navigation interrupted by user!')
        navigator.cancelTask()

    finally:
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            return True
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        return False

    exit(0)


if __name__ == '__main__':
    main()
