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
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml
import os
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

"""
Basic navigation demo to go to pose.
"""
class CamcoResolver(Node):

    def __init__(self):
        super().__init__('camco_resolver')

        self.initial_subscription = self.create_subscription(String,'initial_room',self.initial_listener_callback,10)
        self.initial_subscription  # prevent unused variable warning

        self.goal_subscription = self.create_subscription(String,'goal_room',self.goal_listener_callback,10)
        self.goal_subscription  # prevent unused variable warning

        self.navigator = BasicNavigator()

        pkg_camco_mission = get_package_share_directory('camco_mission')
        self.rooms_yaml_path = os.path.join(pkg_camco_mission, 'room_numbers.yaml')

    def initial_listener_callback(self, msg):
        self.get_logger().info('Initial Room Callback: "%s"' % msg.data)
        
        initial_room = msg.data
        initial_building = initial_building = initial_room.split('-')[0]

        with open(self.rooms_yaml_path, 'r') as roomsResolver:
            rooms = yaml.full_load(roomsResolver)

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = float(rooms.get(initial_building).get(initial_room).get("x"))
        initial_pose.pose.position.y = float(rooms.get(initial_building).get(initial_room).get("y"))
        initial_pose.pose.orientation.z = float(rooms.get(initial_building).get(initial_room).get("z"))
        initial_pose.pose.orientation.w = float(rooms.get(initial_building).get(initial_room).get("w"))
        
        print("Resolved initial pose is: ", initial_pose.__repr__())

        self.navigator.setInitialPose(initial_pose)
    
    def goal_listener_callback(self, msg):
        self.get_logger().info('Goal Room Callback: "%s"' % msg.data)
        
        goal_room = msg.data
        goal_building = goal_room.split('-')[0]

        with open(self.rooms_yaml_path, 'r') as roomsResolver:
            rooms = yaml.full_load(roomsResolver)
        
        self.navigator.waitUntilNav2Active()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(rooms.get(goal_building).get(goal_room).get("x"))
        goal_pose.pose.position.y = float(rooms.get(goal_building).get(goal_room).get("y"))
        goal_pose.pose.orientation.z = float(rooms.get(goal_building).get(goal_room).get("z"))
        goal_pose.pose.orientation.w = float(rooms.get(goal_building).get(goal_room).get("w"))

        print("Resolved goal pose is: ", goal_pose.__repr__())

        self.navigator.goToPose(goal_pose)

        i = 0
        while not self.navigator.isTaskComplete():

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )
        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        




def main():
    rclpy.init()

    camco_resolver = CamcoResolver()

    rclpy.spin(camco_resolver)

    camco_resolver.destroy_node()


if __name__ == '__main__':
    main()