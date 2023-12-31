import os
import rclpy
from rclpy.node import Node
import cv2


from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.image_dir = os.path.join(os.getcwd(), 'images')
        if not os.path.exists(self.image_dir):
            os.mkdir(self.image_dir)
        self.image_count = 0

        self.front_camera_sub = self.create_subscription(
            Image, 
            '/Pioneer3at/camera_front', 
            self.front_camera_callback, 
            10
        )

        self.back_camera_sub = self.create_subscription(
            Image, 
            '/Pioneer3at/camera_back', 
            self.back_camera_callback, 
            10
        )

    def front_camera_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_filename = os.path.join(self.image_dir, f'image_{self.image_count}_front.png')
        print(image_filename)
        cv2.imwrite(image_filename, image)
        self.image_count += 1

    def back_camera_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image_filename = os.path.join(self.image_dir, f'image_{self.image_count}_back.png')
        print(image_filename)
        cv2.imwrite(image_filename, image)
        self.image_count += 1
def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)

if __name__ == '__main__':
    main()
