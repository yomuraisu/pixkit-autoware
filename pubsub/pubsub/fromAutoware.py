import rclpy
from rclpy.node import Node

from autoware_auto_control_msgs.msg import *
from .python_can.msg import *
import can
import time

bus = can.interface.Bus(bustype='socketcan', channel="slcan0", bitrate=500000, app_name='python-can')
# bus = can.Bus('ws://116.80.92.6:54701/', bustype='remote', bitrate=500000)
# bus = can.Bus('ws://localhost:54701/', bustype='remote', bitrate=500000, receive_own_messages=True)

class Subscriber(Node):

    def __init__(self):
        super().__init__('sub')
        self.subscription = self.create_subscription(
            AckermannControlCommand, '/control/command/control_cmd', self.listener_callback, 1000
            # autoware_auto_control_msgs/msg/AckermannControlCommand, '/control/command/control_cmd', self.listener_callback, 10
        )

    def listener_callback(self, recv_topic):
        # self.get_logger().info("Steer_AngleSpeed: %f" % (recv_topic.lateral.steering_tire_rotation_rate))
        # self.get_logger().info("Steer_AngleTarget: %f" % (recv_topic.lateral.steering_tire_angle))
        # self.get_logger().info("Brake_Pedal_Target: %f" % (revc_topic.longitudinal.acceleration))
        # self.get_logger().info("Drive_ThrottlePedalTarget: %f" % (recv_topic.longitudinal.acceleration))
        # self.get_logger().info("Drive_SpeedTarget: %f" % (revc_topic.longitudinal.speed))

        #--------------Steering_Command---------------------
        message_s = Steering_Command()
        
        Steer_EnCtrl = 1
        Steer_AngleSpeed = int(recv_topic.lateral.steering_tire_rotation_rate*100) + 200
        Steer_AngleTarget = int(recv_topic.lateral.steering_tire_angle*1000) + 500

        message_s.setDataFromInt(Steer_EnCtrl, Steer_AngleSpeed, Steer_AngleTarget)
        message_s.toData()
        message_s.view()
        can_msg_s = can.Message(arbitration_id = message_s.msg_id, data= message_s.data, is_extended_id = False)
        for i in range(3):
            bus.send(can_msg_s)
            time.sleep(0.005)

        if recv_topic.longitudinal.acceleration < 0:
            #--------------Brake_Command---------------------
            message_b = Brake_Command()
            
            Brake_EnCtrl = 0
            Brake_Dec = 5 # tekito-
            Brake_Pedal_Target = int(-recv_topic.longitudinal.acceleration*100/1.5)

            message_b.setDataFromInt(Brake_EnCtrl, Brake_Dec, Brake_Pedal_Target)
            message_b.toData()
            message_b.view()
            can_msg_b = can.Message(arbitration_id = message_b.msg_id, data= message_b.data, is_extended_id = False)
            for i in range(3):
                bus.send(can_msg_b)
                time.sleep(0.005)

        

        elif recv_topic.longitudinal.acceleration >= 0:
            #--------------Throttle_Command---------------------
            message_t = Throttle_Command()
            
            Drive_EnCtrl = 0
            Drive_Acc = 5 # tekito-
            Drive_ThrottlePedalTarget = int(recv_topic.longitudinal.acceleration*100/1.5)
            Drive_SpeedTarget = int(recv_topic.longitudinal.speed)

            message_t.setDataFromInt(Drive_EnCtrl, Drive_Acc, Drive_ThrottlePedalTarget, Drive_SpeedTarget)
            message_t.toData()
            message_t.view()
            can_msg_t = can.Message(arbitration_id = message_t.msg_id, data= message_t.data, is_extended_id = False)
            for i in range(3):
                bus.send(can_msg_t)
                time.sleep(0.005)


def main(args=None):
    rclpy.init(args=args)
    mysub = Subscriber()
    rclpy.spin(mysub)
    mysub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()