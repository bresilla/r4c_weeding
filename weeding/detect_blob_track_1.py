#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.object_dict = {}
        self.object_count = 0
        self.max_distance = 50
   
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100:
                continue
            moments = cv2.moments(contour)
            centroid_x = int(moments['m10'] / moments['m00'])
            centroid_y = int(moments['m01'] / moments['m00'])
            object_id = self.get_object_id(centroid_x, centroid_y)
            if object_id is None:
                self.object_dict[self.object_count] = (centroid_x, centroid_y)
                self.object_count += 1
            else:
                self.object_dict[object_id] = (centroid_x, centroid_y)
            cv2.putText(frame, "Object {}".format(object_id), (centroid_x, centroid_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, "Object Count: {}".format(self.object_count), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow('Object Tracker', frame)
        cv2.waitKey(1)
   
    def get_object_id(self, centroid_x, centroid_y):
        for object_id, centroid in self.object_dict.items():
            distance = math.sqrt((centroid_x - centroid[0])**2 + (centroid_y - centroid[1])**2)
            if distance < self.max_distance:
                return object_id
        return None

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()