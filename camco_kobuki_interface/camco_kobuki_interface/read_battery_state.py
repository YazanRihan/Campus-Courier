import rclpy
from rclpy.node import Node
import rclpy.qos
from sensor_msgs.msg import BatteryState
from camco_msgs.srv import ReadBatteryState


class ReadBatteryStateServer(Node):

    def __init__(self):
        super().__init__('read_battery_state_server')
        self.srv = self.create_service(ReadBatteryState, 'read_battery_state', self.read_battery_state_callback)
        self.battery_subscription = self.create_subscription(BatteryState,'sensors/battery_state',self.battery_listner_callback,10)
        self.latest_msg = None
    
    def battery_listner_callback(self, msg):
        self.latest_msg = msg
    
    def read_battery_state_callback(self, request, response):
        response.msg = self.latest_msg
        self.get_logger().info('Incoming read battery state request')
        self.get_logger().info(f'Robot has %{response.msg.percentage}')

        return response
        


def main(args=None):
    rclpy.init(args=args)

    read_battery_state_server = ReadBatteryStateServer()

    rclpy.spin(read_battery_state_server)

    rclpy.shutdown()


if __name__ == '__main__':
    main()