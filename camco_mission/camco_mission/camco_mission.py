import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from camco_msgs.msg import RoomAddress, NavToPoseFeedback
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
from camco_msgs.srv import ReadBatteryState

MIN_BATTERY = 20.0

class CamcoMission(Node):

    def __init__(self):
        super().__init__('camco_mission')

        self.initial_subscription = self.create_subscription(RoomAddress,'initial_address',self.initial_listener_callback,10)
        self.initial_subscription  # prevent unused variable warning

        self.goal_subscription = self.create_subscription(RoomAddress,'goal_address',self.goal_listener_callback,10)
        self.goal_subscription  # prevent unused variable warning

        self.nav_to_pose_feedback_publisher = self.create_publisher(NavToPoseFeedback,
                                                                     'camco/navigate_to_pose/feedback_redirected',
                                                                     10)

        self.navigator = BasicNavigator()

        pkg_camco_mission = get_package_share_directory('camco_mission')
        self.addr_book_yaml_path = os.path.join(pkg_camco_mission, 'address_book.yaml')
        
        self.cli = self.create_client(ReadBatteryState, 'read_battery_state')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('read_battery_state service not available, waiting...')

        self.req = ReadBatteryState.Request()

    
    def send_read_battery_state_request(self):
        self.get_logger().info("Request sent started")
        self.future = self.cli.call_async(self.req)
        self.get_logger().info("Request sent finished")
        rclpy.spin_until_future_complete(self, self.future) 
        self.get_logger().info(f"Request response received, battery is %{self.future.result().msg.percentage}")
        return self.future.result()



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
        self.get_logger().info(f'Goal Address Callback: Building {msg.building} - Room {msg.room}')
        
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

                nav_to_pose_feedback_msg = NavToPoseFeedback()

                self.get_logger().info('Publishing feedback on camco/navigate_to_pose/feedback_redirected')
                nav_to_pose_feedback_msg.current_pose = feedback.current_pose
                nav_to_pose_feedback_msg.navigation_time = feedback.navigation_time
                nav_to_pose_feedback_msg.estimated_time_remaining = feedback.estimated_time_remaining
                nav_to_pose_feedback_msg.number_of_recoveries = feedback.number_of_recoveries
                nav_to_pose_feedback_msg.distance_remaining = feedback.distance_remaining

                self.nav_to_pose_feedback_publisher.publish(nav_to_pose_feedback_msg)

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

    camco_mission = CamcoMission()
    read_battery_state_response = camco_mission.send_read_battery_state_request()

    if read_battery_state_response is not None:
        if read_battery_state_response.msg.percentage < MIN_BATTERY:
            camco_mission.get_logger().error("Insufficient robot charge\nCould not bring node")
            return
        else:
            camco_mission.get_logger().info("Sufficient robot charge\nSpinning node")
    else:
        camco_mission.get_logger().error("Failed to get battery state response")
        return()

    rclpy.spin(camco_mission)

    camco_mission.destroy_node()


if __name__ == '__main__':
    main()