import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float32, Int32
from handy_msgs.msg import Float32Stamped
from cv_bridge import CvBridge
import message_filters
import numpy as np
import matplotlib.pyplot as plt


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("image_subscriber")
        self.cv_bridge = CvBridge()
        self.offset = 1.07

        self.distance = message_filters.Subscriber(self, Float32Stamped, '/gps/distance')
        self.image_back = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_back')
        self.image_front = message_filters.Subscriber(self, Image, '/Pioneer3at/camera_front')
        self.navsatfix = message_filters.Subscriber(self, NavSatFix, '/gps/fix')

        self.blobs_front_pub = self.create_publisher(Int32, "/blobs/front", 10)
        self.blobs_back_pub = self.create_publisher(Int32, "/blobs/back", 10)

        self.pixels_front = []
        self.pixels_back = []

        self.back_sub = message_filters.ApproximateTimeSynchronizer([self.image_back, self.distance], 10, slop=10)
        self.back_sub.registerCallback(self.camera_back)
    
        self.front_sub = message_filters.ApproximateTimeSynchronizer([self.image_front, self.distance], 10, slop=10)
        self.front_sub.registerCallback(self.camera_front)

    def camera_front(self, img, gps):
        # print(gps.latitude)
        # print(gps.longitude)
        camera_front = []
        image = self.cv_bridge.imgmsg_to_cv2(img)
        image, _ = self.blober(image, self.blobs_front_pub)
        cv2.imshow("CAM_FRONT", image)
        cv2.waitKey(1)

    def camera_back(self, img, gps):
        # print(gps.latitude)
        # print(gps.longitude)
        camera_back = []
        image = self.cv_bridge.imgmsg_to_cv2(img)
        image = cv2.flip(image, 1)
        image, _ = self.blober(image, self.blobs_back_pub)
        cv2.imshow("CAM_BACK", image)
        cv2.waitKey(1)

    def blober(self, img, array):
        y, x = int(img.shape[0]/2), 0
        h, w = 150, img.shape[1]
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
        contours, hier = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = 1000
        large_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

        for e in large_contours:
            moments = cv2.moments(e)
            if moments["m00"] != 0:
                x = int(moments["m10"] / moments["m00"])
                y = int(moments["m01"] / moments["m00"])
                cv2.circle(seg_img, (x, y), 5, (0, 255, 0), -1)

        output = cv2.drawContours(seg_img, large_contours, -1, (0, 0, 255), 3)
        return output, seg_img

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    rclpy.shutdown()

if __name__ == "__main__":
    main()