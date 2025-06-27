#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
from ultralytics import YOLO

class MyNode(Node):
    def __init__(self):
        super().__init__('image_display_node')
        
        self.br = CvBridge()
        self.frame = None  # ← Store latest frame here
        
        self.subscription = self.create_subscription(
            Image, 
            '/camera/camera/color/image_raw',  # Change topic name if needed
            self.listener_callback, 
            10)

        # Timer to display image at 30 Hz
        self.timer = self.create_timer(1/30.0, self.display_image)

    def listener_callback(self, msg):
        try:
            self.frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
        
    def display_image(self):
        if self.frame is not None:
            
            model = YOLO("yolov8n.pt")  
            img = self.frame
            results = model(img)[0]
            
            for box in results.boxes:
                if int(box.cls[0]) == 39:  # class 0 = person
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = box.conf[0]
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cx = (x1 + x2) / 2
                    cy = (y1 + y2) / 2
                    cv2.circle(img, (int(cx),int(cy)), 1, (0,0,255), -1 )
                    cv2.putText(img, f'bottle {conf:.2f}', (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Show result
            cv2.imshow("Detection", img)
            key = cv2.waitKey(1)
            if key == ord('q'):
                rclpy.shutdown()


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
