#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from ultralytics import YOLO
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
import torch

class MyNode(Node):
    def __init__(self):
        super().__init__('image_display_node')

        self.model = YOLO("yolov5n.pt").to('cuda')
        # self.model = YOLO("yolov5n.pt")  
        self.br = CvBridge()
        self.frame = None  # ? Store latest frame here
        self.kp = 0.01
        self.new_pub = Twist()
        
        self.publisher_ = self.create_publisher(Int32, '/depth_from_detection', 10)

        self.yaw_publisher = self.create_publisher(Twist, '/yaw_vel', 10)
        
        self.subscription = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw',  # Change topic name if needed
            self.listener_callback, 
            10)
        
        self.depth_frame = None
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)

        # Timer to display image at 30 Hz
        self.timer = self.create_timer(1/50.0, self.display_image)
        # self.timer2 = self.create_timer(1/30.0, self.display_image_Depth)

    def listener_callback(self, msg):
        try:
            print("entered normal callback")
            self.frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            
    def depth_callback(self, msg):
        try:
            print("entered depth callback")
            self.depth_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
        


    def display_image(self):
        
        if self.frame is None or self.depth_frame is None:
            return
        
        if self.frame is not None and self.depth_frame is not None:
            
            img = self.frame
            results = self.model(img)[0]
            
        if self.frame.shape[:2] != self.depth_frame.shape[:2]:
            self.get_logger().warn("RGB and Depth resolutions don't match!")
            
        else:    
            for box in results.boxes:
                print("leaving now")
                if int(box.cls[0]) == 39:  # class 0 = person
                    print("here now")
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = box.conf[0]
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    self.cx = (x1 + x2) / 2
                    self.cy = (y1 + y2) / 2
                    cv2.circle(img, (int(self.cx),int(self.cy)), 1, (0,0,255), -1 )
                    
                    cv2.line(self.frame, (600, 0), (600, 738), (0, 0, 255), 1)
                    cv2.line(self.frame, (670, 0), (670, 738), (0, 0, 255), 1)

                    
                    # cv2.putText(img, f'bottle {conf:.2f}', (x1, y1 - 10),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    # print("here now")
                    depth_val = self.depth_frame[int(self.cy), int(self.cx)]
                    new_val = Int32()
                    new_val.data = int(depth_val)  
                    print("DEPTH OF POINT", depth_val)
                    print("X VALOO", self.cx)
                    self.publisher_.publish(new_val)
                    self.align_bot()
                    

                    # Show result
                    # cv2.imshow("Detection", img)
                    # key = cv2.waitKey(1)
                    # if key == ord('q'):
                    #     rclpy.shutdown()
                else:
                    new_twist = Twist()
                    new_twist.angular.z = 0.0
                    self.yaw_publisher.publish(new_twist)
	
    def limit_val(self,error):
        if error >= 0.2:
            error = 0.2
            return float(error)
        elif error <= -0.2:
            error = -0.2
            return float(error)
    def align_bot(self):
        if self.cx < 600:
            error = 600 - self.cx
            self.bot_left(error)
        elif self.cx > 670:
            error = 670 - self.cy
            self.bot_right(error)
        elif self.cx > 600 and self.cx < 670:
            new_twist = Twist()
            new_twist.angular.z = 0.0
            self.yaw_publisher.publish(new_twist)
            
    def bot_left(self, error):
        #angular z pe plus ya minus
        err_out = 0.0 + error * self.kp
        err_out = self.limit_val(err_out)
        if err_out == None:
            err_out = 0.0
        self.new_pub.angular.z = float(err_out)
        print(err_out)
        self.yaw_publisher.publish(self.new_pub)

    
    def bot_right(self, error):
        #angular z pe plus ya minus
        err_out2 = -0.0 + error * self.kp
        err_out2 = self.limit_val(err_out2)
        if err_out2 == None:
            err_out2 = 0.0
        self.new_pub.angular.z = -(float(err_out2))
        print(err_out2)
        self.yaw_publisher.publish(self.new_pub)



def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
