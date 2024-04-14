import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose

class RedirectMissionStatus(Node):
    def __init__(self):
        super().__init__('redirect_mission_status')
        NavigateToPose.Feedback

        self.nav_to_pose_feedback_subscribtion = self.create_subscription(NavigateToPose.Feedback,
                                                                          'navigate_to_pose/_action/feedback',
                                                                          self.nav_to_pose_feedback_callback,
                                                                          10)
        self.nav_to_pose_feedback_subscribtion  # prevent unused variable warning

        self.nav_to_pose_feedback_publisher = self.create_publisher(NavigateToPose.Feedback,
                                                                     'camco/navigate_to_pose/feedback_redirected',
                                                                     10)

    def nav_to_pose_feedback_callback(self, msg):
        self.get_logger().info('Recevied feedback message from /navigate_to_pose/_action/feedback')
        self.nav_to_pose_feedback_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    redirect_mission_status = RedirectMissionStatus()

    rclpy.spin(redirect_mission_status)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        


