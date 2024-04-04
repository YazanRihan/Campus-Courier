import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from camco_msgs.msg import RoomAddress
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory

class CamcoResolver(Node):

    def __init__(self):
        super().__init__('camco_resolver')

        self.initial_subscription = self.create_subscription(RoomAddress,'initial_address',self.initial_listener_callback,10)
        self.initial_subscription  # prevent unused variable warning

        self.goal_subscription = self.create_subscription(RoomAddress,'goal_address',self.goal_listener_callback,10)
        self.goal_subscription  # prevent unused variable warning

        self.navigator = BasicNavigator()

        pkg_camco_mission = get_package_share_directory('camco_mission')
        self.addr_book_yaml_path = os.path.join(pkg_camco_mission, 'address_book.yaml')

    def initial_listener_callback(self, msg):
        self.get_logger().info(f'Initial Address Callback: Building {msg.building} - Room {msg.room}')
        
        initial_building = msg.building
        initial_room = msg.room

        with open(self.addr_book_yaml_path, 'r') as f:
            addr_book = yaml.full_load(f)
        
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = float(addr_book.get(initial_building).get(initial_room).get("x"))
        initial_pose.pose.position.y = float(addr_book.get(initial_building).get(initial_room).get("y"))
        initial_pose.pose.orientation.z = float(addr_book.get(initial_building).get(initial_room).get("z"))
        initial_pose.pose.orientation.w = float(addr_book.get(initial_building).get(initial_room).get("w"))
        self.get_logger().info(f"Resolved initial pose is: {initial_pose.__repr__()}")

        self.navigator.setInitialPose(initial_pose)
    
    def goal_listener_callback(self, msg):
        self.get_logger().info(f'Initial Address Callback: Building {msg.building} - Room {msg.room}')
        
        goal_building = msg.building
        goal_room = msg.room

        with open(self.addr_book_yaml_path, 'r') as f:
            addr_book = yaml.full_load(f)
        
        self.navigator.waitUntilNav2Active()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(addr_book.get(goal_building).get(goal_room).get("x"))
        goal_pose.pose.position.y = float(addr_book.get(goal_building).get(goal_room).get("y"))
        goal_pose.pose.orientation.z = float(addr_book.get(goal_building).get(goal_room).get("z"))
        goal_pose.pose.orientation.w = float(addr_book.get(goal_building).get(goal_room).get("w"))
        self.get_logger().info(f"Resolved goal pose is: {goal_pose.__repr__()}")

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