import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from handy_msgs.msg import Float32Stamped
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import message_filters
import numpy as np

def approximately_equal(a, b, tolerance=0.0001):
    return abs(a - b) <= tolerance

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.cv_bridge = CvBridge()
        self.offset = 1.07

        self.distance = message_filters.Subscriber(self, Float32Stamped, '/gps/distance')
        self.image_back = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_back')
        self.image_front = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_front')
        self.navsatfix = message_filters.Subscriber(self, NavSatFix, '/gps/fix')

        self.pixels_front_pub = self.create_publisher(Int32, "/pixels/front", 10)
        self.pixels_back_pub = self.create_publisher(Int32, "/pixels/back", 10)

        self.pixels_front = []
        self.pixels_back = []

        self.back_sub = message_filters.ApproximateTimeSynchronizer([self.image_back, self.distance], 10, slop=10)
        self.back_sub.registerCallback(self.camera_back)
    
        self.front_sub = message_filters.ApproximateTimeSynchronizer([self.image_front, self.distance], 10, slop=10)
        self.front_sub.registerCallback(self.camera_front)

    def camera_front(self, img, dist):
        global camera_front
        image = self.cv_bridge.imgmsg_to_cv2(img)
        image, num_white_pixels = self.counter(image, False, "CAM_FRONT")
        self.pixels_front.append((dist.data, num_white_pixels))
        msg = Int32()
        msg.data = 0
        if dist.data > self.offset and len(self.pixels_front) > 1:
            for e in self.pixels_front:
                if approximately_equal(e[0]+self.offset, dist.data, 0.05): break
                self.pixels_front.pop(0)[1]
                print("here")
            pixels_after = self.pixels_front.pop(0)[1]
            msg.data = pixels_after
        self.pixels_front_pub.publish(msg)

    def camera_back(self, img, dist):
        global camera_back
        image = self.cv_bridge.imgmsg_to_cv2(img)
        image = cv2.flip(image, 1)
        image, num_white_pixels = self.counter(image, False, "CAM_BACK")
        self.pixels_back.append((dist.data, num_white_pixels))
        msg = Int32()
        msg.data = 0
        msg.data = num_white_pixels
        self.pixels_back_pub.publish(msg)

    def counter(self, img, display, title):
        y, x = int(img.shape[0]/2), 0
        h, w = 50, img.shape[1]
        roi = img[y:y+h, x:x+w]
        img = roi

        img = cv2.GaussianBlur(img, (9, 9), 0)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        bound_lower = np.array([30, 30, 0])
        bound_upper = np.array([90, 255, 255])

        mask_green = cv2.inRange(hsv_img, bound_lower, bound_upper)

        kernel = np.ones((7,7),np.uint8)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.dilate(mask_green, kernel, iterations=1)
        mask_green = cv2.erode(mask_green, kernel, iterations=1)

        seg_img = cv2.bitwise_and(img, img, mask=mask_green)
        gray_image = cv2.cvtColor(seg_img, cv2.COLOR_BGR2GRAY)
        num_white_pixels = cv2.countNonZero(gray_image)

        if display:
            cv2.imshow(title, seg_img)
            cv2.waitKey(1)

        return seg_img, num_white_pixels

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == "__main__":
    main()