#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, UInt16
from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode, CommandBool

from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer



class MyNode(LifecycleNode):
    def __init__(self):
        super().__init__('node_name')
        print("shi me started")


    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.final_points = 1
        self.Kp = 0.1
        self.Kd = 0.1
        self.depth_val = 0        
        self.velocity_command = Twist()
        
        print("started")

        
        return TransitionCallbackReturn.SUCCESS




    def on_activate(self, state: State) -> TransitionCallbackReturn:

        self.publisher_ = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        self.point_sub = self.create_subscription(UInt16, '/mavros/mission/reached', self.mission_callback, 10)
        self.yaw_subscribe = self.create_subscription(Twist, '/align_vel', self.alignment_callback, 10)
        
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')
        print("idhr ni ara?")
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state):
        self.get_logger().info('Deactivating node...')
        
        if self.subscription:
            self.destroy_subscription(self.subscription)
            self.subscription = None
    
        return TransitionCallbackReturn.SUCCESS    
    
    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS


    def alignment_callback(self, msg):

        self.velocity_command.linear.x = msg.linear.x
        self.velocity_command.linear.y = msg.linear.y
        

    def mission_callback(self, msg):
        if msg.data == self.final_point:
            self.system()
        else:
            print("kya me yaha hu")
            return


    def system(self):
        req = SetMode.Request()
        req.custom_mode = "GUIDED"
        future = self.mode_client.call_async(req)
        future.add_done_callback(self.arm_rover)
        
    def arm_rover(self, future):

        if future.result() and future.result().mode_sent:
            self.get_logger().info('mode call successful')
        else:
            self.get_logger().error('Failed to set mode')
            
        arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        while not arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arming service...')
        req = CommandBool.Request()
        req.value = True
        future = arm_client.call_async(req)
        future.add_done_callback(self.start_follow_behaviour)
        

    
    def start_follow_behaviour(self, future):
        if future.result() and future.result().success:
            self.get_logger().info('arming call successful')
        else:
            self.get_logger().error(f'Failed to arm')
        
        
        #add setpoint logic here
    

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()