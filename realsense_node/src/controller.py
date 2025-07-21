#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import WaypointReached, State

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        print("shi me started")
        self.publisher_ = self.create_publisher(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.subscription = self.create_subscription(
            Int32, '/depth_from_detection', self.listener_callback, 10)
        self.point_sub = self.create_subscription(
            WaypointReached, '/mavros/mission/reached', self.mission_callback, 10)
        self.yaw_subscribe = self.create_subscription(
            Twist, '/yaw_vel', self.alignment_callback, 10)
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)

        self.previous_depth_val = 0.0
        self.current_mode = None

        self.final_points = 1
        self.Kp = 0.1
        self.Kd = 0.1
        self.depth_val = 0
        self.velocity_command = Twist()
        print("started")

        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        print("idhr ni ara?")

    def state_callback(self, msg):
        self.current_mode = msg.mode

    def alignment_callback(self, msg):
        self.velocity_command.angular.z = msg.angular.z

    def mission_callback(self, msg):
        if msg.wp_seq == self.final_points:
            self.system()
        else:
            print("kya me yaha hu")
            return

    def listener_callback(self, msg):
        self.previous_depth_val = self.depth_val
        self.depth_val = msg.data

    def system(self):
        if self.current_mode == "GUIDED":
            self.start_follow_behaviour()
        req = SetMode.Request()
        req.custom_mode = "GUIDED"
        future = self.mode_client.call_async(req)
        self.start_follow_behaviour()

    def limit_val(self, error):
        if error >= 0.2:
            print("idhar aagya")
            print("error ki value is", error)
            error = 0.2
            print("error ki value is NOW", error)
            return float(error)

    def start_follow_behaviour(self):
        self.p_error = self.depth_val * self.Kp
        self.d_error = self.Kd * (self.depth_val - self.previous_depth_val)
        final_vel = self.limit_val(0.0 + self.p_error + self.d_error)
        if final_vel is None:
            final_vel = 0.0
        self.velocity_command.linear.y = float(final_vel)
        self.publisher_.publish(self.velocity_command)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

