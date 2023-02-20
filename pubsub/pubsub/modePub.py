import rclpy
from rclpy.node import Node

from autoware_auto_vehicle_msgs.msg import *
from .python_can.msg import *
import can

bus = can.interface.Bus(bustype='socketcan', channel="slcan0", bitrate=500000, app_name='python-can')
# bus = can.Bus('ws://116.80.92.6:54701/', bustype='remote', bitrate=500000)
# bus = can.Bus('ws://localhost:54701/', bustype='remote', bitrate=500000, receive_own_messages=True)

class Publisher_mode(Node):
    def __init__(self):
        super().__init__('pub_mode')
        self.publisher = self.create_publisher(
            ControlModeReport, '/vehicle/status/control_mode', 10
        )
        timer_period = 0.05
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.message_mode = ControlModeReport()

    def timer_callback(self):
        self.message_mode.stamp = self.get_clock().now().to_msg()
        self.message_mode.mode = 1
        self.publisher.publish(self.message_mode)
        # self.get_logger().info("stamp: %f , mode: %f" % (self.message_mode.stamp ,self.message_mode.mode)) 


def main(args=None):
    rclpy.init(args=args)
    mypub_mode = Publisher_mode()
    rclpy.spin(mypub_mode)
    mypub_mode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()