import rclpy
from rclpy.node import Node

from autoware_auto_vehicle_msgs.msg import *
from .python_can.msg import *
import can
import time

bus = can.interface.Bus(bustype='socketcan', channel="slcan0", bitrate=500000, app_name='python-can')
# bus = can.Bus('ws://116.80.92.6:54701/', bustype='remote', bitrate=500000)
# bus = can.Bus('ws://localhost:54701/', bustype='remote', bitrate=500000, receive_own_messages=True)


class Publisher_velo(Node):
    def __init__(self):
        super().__init__('pub_velo')
        self.publisher = self.create_publisher(
            VelocityReport, '/vehicle/status/velocity_status', 10
        )
        
        self.message_velo = VelocityReport()
        
    def callback_velo(self, steering, velocity):
        self.message_velo.header.stamp = self.get_clock().now().to_msg()
        self.message_velo.header.frame_id = 'base_link'
        self.message_velo.longitudinal_velocity = velocity
        self.message_velo.lateral_velocity = 0.0
        self.message_velo.heading_rate = steering/100
        self.publisher.publish(self.message_velo)

class Publisher_st(Node):
    def __init__(self):
        super().__init__('pub_st')
        self.publisher = self.create_publisher(
            SteeringReport, '/vehicle/status/steering_status', 10
        )
        
        self.message_st = SteeringReport()
        
    def callback_st(self, steering):
        self.message_st.stamp = self.get_clock().now().to_msg()
        self.message_st.steering_tire_angle = steering
        self.publisher.publish(self.message_st)


def main(args=None):
    rclpy.init(args=args)

    mypub_velo = Publisher_velo()
    mypub_st = Publisher_st()

    steering_can = Steering_Report()
    throttle_can = Throttle_Report()
    brake_can = Brake_Report()
    speed_can = WheelSpeed_Report()

    steering = 0
    velocity =0

    while True:
        print("==============================================")
        # CANメッセージを受信
        start_time = time.time()
        while time.time() - start_time < 0.05 : # 0.01sec間モニター
            recv_msg = bus.recv(timeout=1) # canメッセージを受け取る
            if recv_msg != None: # メッセージがあるとき
                if recv_msg.arbitration_id == 0x502: 
                    print(recv_msg.data)
                    steering_can.setDataFromCANMessage(recv_msg.data)
                    print(steering_can.Steer_AngleActual)
                    steering = (steering_can.Steer_AngleActual - 500)/100 # canデータをautowareの車両のスケールに合わせる
                    # rclpy.spin_once(mypub_velo)
                    # mypub_velo.callback_velo(steering, velocity)
                    # rclpy.spin_once(mypub_st)
                    mypub_st.callback_st(steering)


                # elif recv_msg.arbitration_id == 0x500: 
                #     print(recv_msg.data)
                #     throttle_can.setDataFromCANMessage(recv_msg.data)
                #     print(throttle_can.Drive_ThrottlePedalActual)
                #     throttle = throttle_can.Drive_ThrottlePedalActual*100/1.5 # canデータをautowareの車両のスケールに合わせる
                    

                # elif recv_msg.arbitration_id == 0x501: 
                #     print(recv_msg.data)
                #     brake_can.setDataFromCANMessage(recv_msg.data)
                #     print(brake_can.Brake_PedalActual)
                #     throttle = -brake_can.Brake_PedalActual*1.5/100 # canデータをautowareの車両のスケールに合わせる
                    
                elif recv_msg.arbitration_id == 0x506:
                    print(recv_msg.data)
                    speed_can.setDataFromCANMessage(recv_msg.data)
                    print(speed_can.WheelSpeedFL)
                    velocity = speed_can.WheelSpeedFL*3.6 # km/h
                    mypub_velo.callback_velo(steering, velocity)
                    

    mypub_velo.destroy_node()
    mypub_st.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()