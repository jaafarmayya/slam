#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SlamImageSubscriber(Node):
    def __init__(self):
        super().__init__('slam_image_subscriber')
        self.bridge = CvBridge()
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_cb,
            10)
        self.get_logger().info('slam_image_subscriber started')

    def image_cb(self, msg: Image):
        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
      

def main(args=None):
    rclpy.init(args=args)
    node = SlamImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
