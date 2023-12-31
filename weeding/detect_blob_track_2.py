#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.trackers = []
        self.object_count = 0
        self.color = (0, 255, 0)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        current_boxes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100:
                continue
            moments = cv2.moments(contour)
            centroid_x = int(moments['m10'] / moments['m00'])
            centroid_y = int(moments['m01'] / moments['m00'])
            current_box = (centroid_x - 50, centroid_y - 50, 100, 100) # create a bounding box around the object
            current_boxes.append(current_box)

        new_trackers = []
        for box in current_boxes:
            is_new_object = True
            for tracker in self.trackers:
                _, tracked_box = tracker.update(frame)
                dist = np.linalg.norm(np.array((box[0]+50, box[1]+50)) - np.array((tracked_box[0]+50, tracked_box[1]+50)))
                if dist < 20: # if the current box is within 20 pixels of a tracked box, it's likely the same object
                    is_new_object = False
                    new_trackers.append(tracker)
                    break

            if is_new_object:
                self.object_count += 1
                self.color = (0, 255, 0) # set color to green for new objects
                tracker = cv2.TrackerCSRT_create()
                tracker.init(frame, box)
                new_trackers.append(tracker)

        self.trackers = new_trackers

        for tracker in self.trackers:
            _, box = tracker.update(frame)
            box = tuple(map(int, box))
            cv2.rectangle(frame, box, self.color, 2)

        cv2.putText(frame, "Object Count: {}".format(self.object_count), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow('Object Tracker', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()